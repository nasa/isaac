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

// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>

// Tf2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// FSW includes
#include <config_reader/config_reader.h>
#include <isaac_util/isaac_names.h>

// Messages
#include <isaac_hw_msgs/AirQuality.h>

// STL includes
#include <string>

namespace gazebo {

// This class is a plugin that calls the GNC autocode to predict
// the force to be applied to the rigid body
class GazeboModelPluginAirQuality : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginAirQuality() :
    FreeFlyerModelPlugin("air_quality", ""), rate_(0.0) {}

  ~GazeboModelPluginAirQuality() {}

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh,
    physics::ModelPtr model, sdf::ElementPtr sdf) {
    // Read the config file
    config_reader::ConfigReader config_params;

    // Set the config path to ISAAC
    char *path;
    if ((path = getenv("ISAAC_CONFIG_DIR")) == NULL)
      LOG(FATAL) << "Could not find the config path.";
    config_params.SetPath(path);

    config_params.AddFile("hw/air_quality.config");
    if (!config_params.ReadFiles())
      LOG(FATAL) << "Failed to read config files.";


    if (!config_params.GetReal("air_quality_rate", &rate_))
      LOG(FATAL) << "Could not read the heat_cam_rate parameter.";

    if (!config_params.GetReal("min_quality", &min_quality_))
      LOG(FATAL) << "Could not read the min quality parameter.";
    if (!config_params.GetReal("max_quality", &max_quality_))
      LOG(FATAL) << "Could not read the max quality parameter.";
    if (!config_params.GetReal("ambient_quality", &ambient_quality_))
      LOG(FATAL) << "Could not read the ambient quality parameter.";

    if (ambient_quality_ < min_quality_)
      LOG(FATAL) << "The ambient quality must be no less than the min radiance.";
    if (max_quality_ < ambient_quality_)
      LOG(FATAL) << "The ambient quality must be no more than the max radiance.";

    // Read the hot spots
    config_reader::ConfigReader::Table mat(&config_params, "air_spots");
    int num_vals =  mat.GetSize();
    int num_cols = 6;
    int num_rows = num_vals/num_cols;
    if (num_vals == 0 || num_vals != num_rows * num_cols)
      LOG(FATAL) << "Expecting the number of values in air_spots to be a positive multiple of "
                 << num_cols;

    quality_spots_ = Eigen::MatrixXd(num_rows, num_cols);
    int val_count = 1;  // indices start from 1 in arrays in the config file
    for (int row = 0; row < num_rows; row++) {
      for (int col = 0; col < num_cols; col++) {
        if (!mat.GetReal(val_count, &quality_spots_(row, col)))
          LOG(FATAL) << "Could not read air_spots";
        val_count++;
      }
    }
    config_params.Close();

    // Create publishers
    pub_air_quality_ = nh->advertise<isaac_hw_msgs::AirQuality>(
      TOPIC_HARDWARE_AIR_QUALITY, 1);

    // Called before each iteration of simulated world update
    timer_ = nh->createTimer(ros::Rate(rate_),
      &GazeboModelPluginAirQuality::TimerCallback, this, false, true);
  }

  // Called on simulation reset
  void Reset() {}

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

  // Called on every discrete time tick in the simulated world
  void TimerCallback(ros::TimerEvent const& event) {
    // Declare the air quality message
    isaac_hw_msgs::AirQuality air_quality_msg;
    air_quality_msg.header.stamp = ros::Time::now();

    // Get the current pose
    #if GAZEBO_MAJOR_VERSION > 7
    Eigen::Vector3d P(GetModel()->WorldPose().Pos().X(),
                      GetModel()->WorldPose().Pos().Y(),
                      GetModel()->WorldPose().Pos().Z());
    #else
    Eigen::Vector3d P(GetModel()->GetWorldPose().pos.x,
                      GetModel()->GetWorldPose().pos.y,
                      GetModel()->GetWorldPose().pos.z);
    #endif
    // Start with the default quality
    double quality = ambient_quality_;

    // See if we are around a high or low quality spot
    int num_spots = quality_spots_.rows();
    for (int spot_iter = 0; spot_iter < num_spots; spot_iter++) {
      double x0             = quality_spots_(spot_iter, 0);
      double y0             = quality_spots_(spot_iter, 1);
      double z0             = quality_spots_(spot_iter, 2);
      double inner_radius   = quality_spots_(spot_iter, 3);
      double outer_radius   = quality_spots_(spot_iter, 4);
      double delta_quality  = quality_spots_(spot_iter, 5);

      double dist = Eigen::Vector3d(P[0] - x0, P[1] - y0, P[2] - z0).norm();
      if (dist > outer_radius) continue;

      // Make the quality increase from 0 to 1 between these radii
      double unit_quality = s_function(inner_radius, outer_radius, dist);

      // Radiance must decrease
      unit_quality = 1.0 - unit_quality;

      // Adjust the quality
      quality = ambient_quality_ + delta_quality * unit_quality;
    }

    // Clamp to min and max quality
    if (quality < min_quality_)
      quality = min_quality_;
    if (quality > max_quality_)
      quality = max_quality_;

    // Publish the message
    air_quality_msg.air_quality = quality;
    pub_air_quality_.publish(air_quality_msg);
  }

 private:
  double rate_;
  ros::Publisher pub_air_quality_;
  ros::Timer timer_;

  double min_quality_, max_quality_, ambient_quality_;
  Eigen::MatrixXd quality_spots_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginAirQuality)

}   // namespace gazebo
