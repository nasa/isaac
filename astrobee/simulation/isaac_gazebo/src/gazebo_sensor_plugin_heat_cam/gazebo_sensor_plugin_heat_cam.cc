/* Copyright (c) 2021, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <ros/ros.h>
#include <config_reader/config_reader.h>
#include <astrobee_gazebo/astrobee_gazebo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/CommandConstants.h>

#include <isaac_util/isaac_names.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <string>
#include <cmath>

namespace gazebo {

// An increasing function returning values between 0 and 1. At x = a
// it returns values close to 0, and at x = b it returns values close
// to 1. In between, it smoothly transitions from 0 to 1. As x keeps
// on growing the values get asymptotically closer to 1, and as x
// decreases they get asymptotically closer to 0.
double s_function(double a, double b, double x) {
  if (a >= b)
    LOG(FATAL) << "Incorrect use of s_function. Must have a < b.";

  x = 2.0 * (x - a)/(b - a) - 1.0;  // x is now distributed around [-1, 1] but can go beyond
  x = 3.0 * x;
  double y = atan(x);  // between -pi/2 and p/2
  return (y + M_PI/2.0)/M_PI;  // between 0 and 1
}

class GazeboSensorPluginHeatCam : public FreeFlyerSensorPlugin {
 public:
  // Note that we put the heat cam in the nav cam frame
  GazeboSensorPluginHeatCam() :
    FreeFlyerSensorPlugin("heat_cam", "nav_cam", false),
    continuousPictureTaking_(false), takeSinglePicture_(false), rate_(0.0) {}

  ~GazeboSensorPluginHeatCam() {
    if (depth_update_)
    #if GAZEBO_MAJOR_VERSION > 7
      depth_update_.reset();
    #else
      camera_->DisconnectNewRGBPointCloud(depth_update_);
    #endif
  }

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle *nh,
                    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginHeatCam requires a depth camera sensor.\n";
      return;
    }

    // Get a link to the depth camera
    camera_ = sensor_->DepthCamera();
    if (!camera_) {
      gzerr << "GazeboSensorPluginHeatCam cannot get rendering object.\n";
      return;
    }

    // Look at the intensity component of the depth camera,
    // which we will call the "amplitude"
    // Check that we have a mono camera
    if (camera_->ImageFormat() != "L8")
      LOG(FATAL) << "Camera format must be L8";

    // Create subscriber to DDS commands though which the heat cam will be controlled
    dds_cmd_sub_ = nh->subscribe(TOPIC_COMMUNICATIONS_DDS_COMMAND, 10,
                                 &GazeboSensorPluginHeatCam::DdsCmdCallback, this);

    // Create publishers for heat cam image, pose, and camera info
    pub_heat_cam_image_ = nh->advertise<sensor_msgs::Image>
      (TOPIC_HARDWARE_HEAT_CAM, 2,
       boost::bind(&GazeboSensorPluginHeatCam::ToggleCallback, this),
       boost::bind(&GazeboSensorPluginHeatCam::ToggleCallback, this));
    pub_heat_cam_pose_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_HEAT_CAM_SIM_POSE, 10);
    pub_heat_cam_info_ = nh->advertise<sensor_msgs::CameraInfo>(TOPIC_HEAT_CAM_SIM_INFO, 10);

    // Read the config file
    config_reader::ConfigReader config_params;

    // Set the config path to ISAAC
    char *path;
    if ((path = getenv("ISAAC_CONFIG_DIR")) == NULL)
      LOG(FATAL) << "Could not find the config path.";
    config_params.SetPath(path);

    config_params.AddFile("hw/heat_cam.config");
    if (!config_params.ReadFiles())
      LOG(FATAL) << "Failed to read config files.";

    if (!config_params.GetReal("heat_cam_rate", &rate_))
      LOG(FATAL) << "Could not read the heat_cam_rate parameter.";

    bool cp;
    if (config_params.GetBool("continuous_picture_taking", &cp))
      continuousPictureTaking_ = cp;

    if (!config_params.GetReal("min_radiance", &min_radiance_))
      LOG(FATAL) << "Could not read the min radiance parameter.";
    if (!config_params.GetReal("max_radiance", &max_radiance_))
      LOG(FATAL) << "Could not read the max radiance parameter.";
    if (!config_params.GetReal("ambient_radiance", &ambient_radiance_))
      LOG(FATAL) << "Could not read the ambient radiance parameter.";

    if (ambient_radiance_ < min_radiance_)
      LOG(FATAL) << "The ambient radiance must be no less than the min radiance.";
    if (max_radiance_ < ambient_radiance_)
      LOG(FATAL) << "The ambient radiance must be no more than the max radiance.";

    // Read the hot spots
    config_reader::ConfigReader::Table mat(&config_params, "hot_spots");
    int num_vals =  mat.GetSize();
    int num_cols = 6;
    int num_rows = num_vals/num_cols;
    if (num_vals == 0 || num_vals != num_rows * num_cols)
      LOG(FATAL) << "Expecting the number of values in hot_spots to be a positive multiple of "
                 << num_cols;

    hot_spots_ = Eigen::MatrixXd(num_rows, num_cols);
    int val_count = 1;  // indices start from 1 in arrays in the config file
    for (int row = 0; row < num_rows; row++) {
      for (int col = 0; col < num_cols; col++) {
        if (!mat.GetReal(val_count, &hot_spots_(row, col)))
          LOG(FATAL) << "Could not read hot_spots";
        val_count++;
      }
    }

    config_params.Close();

    // Toggle if the camera is active or not
    ToggleCallback();
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(ros::NodeHandle *nh) {
    // Listen to the point cloud
    depth_update_ = camera_->ConnectNewRGBPointCloud
      (boost::bind(&GazeboSensorPluginHeatCam::PointCloudCallback, this, _1, _2, _3, _4, _5));
  }

  // Turn camera on or off based on topic subscription
  void ToggleCallback() {
    if (pub_heat_cam_image_.getNumSubscribers() > 0 && rate_ > 0) {
      sensor_->SetUpdateRate(rate_);
      sensor_->SetActive(true);
    } else {
      sensor_->SetUpdateRate(0.0001);
      sensor_->SetActive(false);
    }
  }

  // TODO(oalexan1): Factor this out!
  // Out of string: "{"name": "turnOnContinuousPictureTaking"}"
  // collect the part in the second set of quotes.
  // Some honest json parsing could be used here.
  std::string parseJsonStr(std::string const& json_str) {
    size_t start = 0;
    size_t colon_pos = json_str.find(":", start);
    if (colon_pos == std::string::npos) {
      return "";
    }
    size_t quote1_pos = json_str.find("\"", colon_pos + 1);
    if (quote1_pos == std::string::npos) {
      return "";
    }
    size_t quote2_pos = json_str.find("\"", quote1_pos + 1);

    if (quote2_pos == std::string::npos) {
      return "";
    }

    std::string parsed = json_str.substr(quote1_pos + 1, quote2_pos - quote1_pos - 1);

    return parsed;
  }

  // Called when a dds command is received. Process only guest heatence
  // heat cam control commands.
  void DdsCmdCallback(ff_msgs::CommandStamped const& cmd) {
    // Process only guest science commands
    if (cmd.cmd_name != ff_msgs::CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE) {
      // Only process custom heat cam commands
      return;
    }

    if (cmd.args.size() != 2) {
      // Custom heat cam commands have two arguments
      return;
    }

    if (cmd.args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
        cmd.args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ) {
      return;
    }

    std::string app_name = cmd.args[0].s.data();

    // Process only heat cam commands
    if (app_name != "gov.nasa.arc.irg.astrobee.heat_cam_image") {
      return;
    }

    std::string json_str = cmd.args[1].s.data();
    ROS_INFO_STREAM("Received command: " << json_str);

    std::string action = parseJsonStr(json_str);
    if (action == "")
      return;

    // Record the desired intention. Use a lock.
    {
      const std::lock_guard<std::mutex> lock(heat_cam_image_lock);
      if (action == "takeSinglePicture") {
        takeSinglePicture_ = true;
        continuousPictureTaking_ = false;
      } else if (action == "turnOnContinuousPictureTaking") {
        takeSinglePicture_ = false;
        continuousPictureTaking_ = true;
      } else if (action == "turnOffContinuousPictureTaking") {
        takeSinglePicture_ = false;
        continuousPictureTaking_ = false;
      } else {
        LOG(FATAL) << "Unknown heat_cam command: " << action;
      }
    }

    return;
  }

  // Publish the heat cam cloud and other data
  void PointCloudCallback(const float *depth_data, unsigned int width, unsigned int height,
                          unsigned int len, const std::string & type) {
    // Quickly record the current time and current pose before doing other computations
    ros::Time curr_time = ros::Time::now();

    // Publish the heat cam pose
    #if GAZEBO_MAJOR_VERSION > 7
    Eigen::Affine3d sensor_to_world = SensorToWorld(GetModel()->WorldPose(), sensor_->Pose());
    #else
    Eigen::Affine3d sensor_to_world = SensorToWorld(GetModel()->GetWorldPose(), sensor_->Pose());
    #endif
    heat_cam_pose_msg_.header.frame_id = GetFrame();
    heat_cam_pose_msg_.header.stamp = curr_time;  // it is very important to get the time right
    heat_cam_pose_msg_.pose.position.x = sensor_to_world.translation().x();
    heat_cam_pose_msg_.pose.position.y = sensor_to_world.translation().y();
    heat_cam_pose_msg_.pose.position.z = sensor_to_world.translation().z();
    Eigen::Quaterniond q(sensor_to_world.rotation());
    heat_cam_pose_msg_.pose.orientation.w = q.w();
    heat_cam_pose_msg_.pose.orientation.x = q.x();
    heat_cam_pose_msg_.pose.orientation.y = q.y();
    heat_cam_pose_msg_.pose.orientation.z = q.z();
    pub_heat_cam_pose_.publish(heat_cam_pose_msg_);

    // Publish the heat cam intrinsics
    heat_cam_info_msg_.header.frame_id = GetFrame();
    heat_cam_info_msg_.header.stamp = curr_time;  // it is very important to get the time right
    FillCameraInfo(sensor_->DepthCamera(), heat_cam_info_msg_);  // fill in from the camera pointer
    pub_heat_cam_info_.publish(heat_cam_info_msg_);

    // Do not publish the image unless specifically told to
    if (!continuousPictureTaking_ && !takeSinglePicture_) {
      return;
    }

    // Compute the grayscale radiance (scaled to 0 to 255).
    cv::Mat gray_img(camera_->ImageHeight(), camera_->ImageWidth(), CV_8UC1, cv::Scalar(0));
    int count = -1;
    for (int row = 0; row < gray_img.rows; row++) {
      for (int col = 0; col < gray_img.cols; col++) {
        count++;
        double x     = depth_data[4*count + 0];
        double y     = depth_data[4*count + 1];
        double z     = depth_data[4*count + 2];
        // double inten = depth_data[4*count + 3];  // Gazebo always sets this to 1.

        // Skip invalid measurements
        if (x == 0 && y == 0 && z == 0)
          continue;

        // Convert the current depth measurement to world coordinates
        Eigen::Vector3d P(x, y, z);
        P = sensor_to_world * P;

        // Start with the default radiance
        double radiance = ambient_radiance_;

        // See if we are around a hot or cold spot
        int num_spots = hot_spots_.rows();
        for (int spot_iter = 0; spot_iter < num_spots; spot_iter++) {
          double x0             = hot_spots_(spot_iter, 0);
          double y0             = hot_spots_(spot_iter, 1);
          double z0             = hot_spots_(spot_iter, 2);
          double inner_radius   = hot_spots_(spot_iter, 3);
          double outer_radius   = hot_spots_(spot_iter, 4);
          double delta_radiance = hot_spots_(spot_iter, 5);

          double dist = Eigen::Vector3d(P[0] - x0, P[1] - y0, P[2] - z0).norm();
          if (dist > outer_radius) continue;

          // Make the radiance increase from 0 to 1 between these radii
          double unit_radiance = s_function(inner_radius, outer_radius, dist);

          // Radiance must decrease
          unit_radiance = 1.0 - unit_radiance;

          // Adjust the radiance
          radiance = ambient_radiance_ + delta_radiance * unit_radiance;
        }

        // Clamp to min and max radiance
        if (radiance < min_radiance_)
          radiance = min_radiance_;
        if (radiance > max_radiance_)
          radiance = max_radiance_;

        // Scale to 0, 255
        gray_img.at<uchar>(row, col)
          = round(255.0 * (radiance - min_radiance_) / (max_radiance_ - min_radiance_));
      }
    }

    // Colorize the radiance
    cv::Mat color_img;
    cv::applyColorMap(gray_img, color_img, cv::COLORMAP_JET);

    // Publish the radiance
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                   "bgr8", color_img).toImageMsg();
    msg->header.stamp.sec  = sensor_->LastMeasurementTime().sec;
    msg->header.stamp.nsec = sensor_->LastMeasurementTime().nsec;
    msg->header.stamp = curr_time;
    pub_heat_cam_image_.publish(msg);

    if (takeSinglePicture_) {
      // Done taking a single picture. Use a lock to change this flag.
      const std::lock_guard<std::mutex> lock(heat_cam_image_lock);
      takeSinglePicture_ = false;
    }

    return;
  }

 private:
  geometry_msgs::PoseStamped heat_cam_pose_msg_;
  sensor_msgs::CameraInfo heat_cam_info_msg_;

  bool continuousPictureTaking_;
  bool takeSinglePicture_;
  std::mutex heat_cam_image_lock;

  ros::Publisher pub_heat_cam_image_;
  ros::Publisher pub_heat_cam_pose_;
  ros::Publisher pub_heat_cam_info_;
  ros::Subscriber dds_cmd_sub_;
  sensors::DepthCameraSensorPtr sensor_;
  rendering::DepthCameraPtr camera_;
  event::ConnectionPtr depth_update_;
  double rate_;

  double min_radiance_, max_radiance_, ambient_radiance_;
  Eigen::MatrixXd hot_spots_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginHeatCam)

}   // namespace gazebo
