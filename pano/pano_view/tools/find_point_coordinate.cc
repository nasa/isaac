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

// Command line flags
#include <gflags/gflags.h>
#include <gflags/gflags_completions.h>


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// Import messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ff_msgs/EkfState.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

// Shared project includes
#include <config_reader/config_reader.h>
#include <camera/camera_params.h>
#include <inspection/camera_projection.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pano_view/point_cloud_intersect.h>
#include <pano_view/mesh_intersect.h>

// json file reader lib
#include <jsoncpp/json/allocator.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/value.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

// Parameters
DEFINE_string(camera, "sci_cam", "Camera name.");
DEFINE_string(depth, "haz", "Depth camera name.");
DEFINE_string(bag_name, "", "Bagname where image can be found. Make sure it has haz cam and ground truth");
DEFINE_string(mesh_name, "", "Meshfile path.");
DEFINE_string(json_config, "", "json file with configure data.");

// Bagfile topics
DEFINE_string(image_info_topic,   "/hw/cam_sci_info",     "Camera info topic name.");
DEFINE_string(depth_cam_topic,    "/hw/depth_haz/points", "Point Cloud topic name.");
DEFINE_string(ground_truth_topic, "/gnc/ekf",             "Robot pose topic name.");

void extractTopicsBag(const std::string bagname, const double timestamp, const std::string image_info_topic,
                      const std::string depth_cam_topic, const std::string ground_truth_topic,
                      sensor_msgs::CameraInfo& camera_info_out, sensor_msgs::PointCloud2& depth_cam_out,
                      geometry_msgs::Pose& ground_truth_out) {
  // Read ground truth from bag
  rosbag::Bag bag;
  bag.open(FLAGS_bag_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(depth_cam_topic);
  topics.push_back(ground_truth_topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  double timestamp_dist_camera_info = std::numeric_limits<double>::max();
  double timestamp_dist_depth_cam = std::numeric_limits<double>::max();
  double timestamp_dist_ground_truth = std::numeric_limits<double>::max();
  BOOST_FOREACH(rosbag::MessageInstance const m, view) {
    // Read camera info
    if (m.getTopic() == image_info_topic || ("/" + m.getTopic() == image_info_topic)) {
      sensor_msgs::CameraInfo::ConstPtr camera_info = m.instantiate<sensor_msgs::CameraInfo>();
      if (camera_info != NULL && abs(camera_info->header.stamp.toSec() - timestamp) < timestamp_dist_depth_cam) {
        camera_info_out = *camera_info;
        timestamp_dist_camera_info = abs(camera_info->header.stamp.toSec() - timestamp);
      }
    }
    // Read depth cam points
    if (m.getTopic() == depth_cam_topic || ("/" + m.getTopic() == depth_cam_topic)) {
      sensor_msgs::PointCloud2::ConstPtr depth_cam = m.instantiate<sensor_msgs::PointCloud2>();
      if (depth_cam != NULL && abs(depth_cam->header.stamp.toSec() - timestamp) < timestamp_dist_depth_cam) {
        depth_cam_out = *depth_cam;
        timestamp_dist_depth_cam = abs(depth_cam->header.stamp.toSec() - timestamp);
      }
    }
    // Read ground truth data
    if (m.getTopic() == ground_truth_topic || ("/" + m.getTopic() == ground_truth_topic)) {
      ff_msgs::EkfState::ConstPtr ground_truth = m.instantiate<ff_msgs::EkfState>();
      if (ground_truth != NULL  && abs(ground_truth->header.stamp.toSec() - timestamp) < timestamp_dist_ground_truth) {
        ground_truth_out = ground_truth->pose;
        timestamp_dist_ground_truth = abs(ground_truth->header.stamp.toSec() - timestamp);
      }
    }
  }
  std::cout << "Closest timestamp camera info : " << timestamp_dist_camera_info
            << "Closest timestamp depth: " << timestamp_dist_depth_cam
            << " Closest timestamp pose: " << timestamp_dist_ground_truth << std::endl;
  bag.close();
}

int main(int argc, char** argv) {
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun pano_view find_point_coordinate <opts>");
  google::SetVersionString("0.1.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Read parameters from config file
  std::ifstream input_file(FLAGS_json_config);
  if (!input_file.is_open()) {
    std::cerr << "Failed to open file: " << FLAGS_json_config << std::endl;
    return 1;
  }
  Json::Reader reader;
  Json::Value json;
  bool success = reader.parse(input_file, json);
  if (!success) {
    std::cerr << "Failed to parse JSON data: " << reader.getFormattedErrorMessages() << std::endl;
    return 1;
  }

  double timestamp = json["timestamp"].asDouble();
  std::string camera_name = json["camera"].asString();
  int coord_x = json["coord"]["x"].asInt();
  int coord_y = json["coord"]["y"].asInt();

  // Read transforms
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("transforms.config");
  if (!config.ReadFiles()) LOG(FATAL) << "Failed to read config files.";

  // Read Transform
  Eigen::Affine3d transform_nav_to_depth;
  std::string transform_str = "nav_cam_to_" + FLAGS_depth + "_cam_transform";
  if (!msg_conversions::config_read_transform(&config, transform_str.c_str(), &transform_nav_to_depth)) {
    LOG(FATAL) << "Unspecified transform: " << transform_str << " for robot: "
               << getenv("ASTROBEE_ROBOT") << "\n";
    return 1;
  }
  Eigen::Affine3d transform_nav_to_cam;
  transform_str = "nav_cam_to_" + camera_name + "_transform";
  if (!msg_conversions::config_read_transform(&config, transform_str.c_str(), &transform_nav_to_cam)) {
    LOG(FATAL) << "Unspecified transform: " << transform_str << " for robot: "
               << getenv("ASTROBEE_ROBOT") << "\n";
    return 1;
  }
  Eigen::Affine3d transform_body_to_nav;
  transform_str = "nav_cam_transform";
  if (!msg_conversions::config_read_transform(&config, transform_str.c_str(), &transform_body_to_nav)) {
    LOG(FATAL) << "Unspecified transform: " << transform_str << " for robot: "
               << getenv("ASTROBEE_ROBOT") << "\n";
    return 1;
  }
  Eigen::Affine3d transform_body_to_haz;
  transform_str = "haz_cam_transform";
  if (!msg_conversions::config_read_transform(&config, transform_str.c_str(), &transform_body_to_haz)) {
    LOG(FATAL) << "Unspecified transform: " << transform_str << " for robot: "
               << getenv("ASTROBEE_ROBOT") << "\n";
    return 1;
  }

  Eigen::MatrixXd M(4, 4);
  Eigen::Affine3d haz_cam_depth_to_image_trans;
  transform_str = FLAGS_depth + "_cam_depth_to_image_transform";
  config_reader::ConfigReader::Table mat(&config, transform_str.c_str());
  int count = 0;
  for (int row = 0; row < M.rows(); row++) {
    for (int col = 0; col < M.cols(); col++) {
      count++;  // note that the count stats from 1
      if (!mat.GetReal(count, &M(row, col))) {
        LOG(FATAL) << "Could not read value of " << transform_str
                   << " for robot: " << getenv("ASTROBEE_ROBOT");
        return 1;
      }
    }
  }
  haz_cam_depth_to_image_trans.matrix() = M;

  // Initialize camera view
  Eigen::Affine3d transform_body_to_cam = transform_body_to_nav * transform_nav_to_cam;
  geometry_msgs::Transform::ConstPtr msg_pointer(
    new geometry_msgs::Transform(msg_conversions::eigen_transform_to_ros_transform(transform_body_to_cam)));
  inspection::CameraView camera(camera_name, 2.0, 0.19, msg_pointer);

  // Extract topics from bagfile
  std::string image_info_topic = FLAGS_image_info_topic;
  std::string depth_cam_topic = FLAGS_depth_cam_topic;
  std::string ground_truth_topic = FLAGS_ground_truth_topic;
  sensor_msgs::CameraInfo image_info;
  sensor_msgs::PointCloud2 depth_cam, point_cloud_world;
  geometry_msgs::Pose ground_truth;
  extractTopicsBag(FLAGS_bag_name, timestamp, image_info_topic, depth_cam_topic, ground_truth_topic, image_info,
                   depth_cam, ground_truth);
  camera.SetH(image_info.height);
  camera.SetW(image_info.width);

  // Figure out the vector to the target based on camera parameters
  Eigen::Vector3d vector;
  camera.GetVectorFromCamXY(ground_truth, coord_x, coord_y, vector);

  // Transform Point Cloud to world reference frame + fix scaling
  Eigen::Affine3d transform_world_to_depth =
    msg_conversions::ros_pose_to_eigen_transform(ground_truth) * transform_body_to_nav * transform_nav_to_depth;

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(depth_cam, pcl_cloud);
  for (auto& pcl_point : pcl_cloud.points) {
    Eigen::Vector3d X(pcl_point.x, pcl_point.y, pcl_point.z);
    X = transform_world_to_depth * haz_cam_depth_to_image_trans * X;
    pcl_point.x = X[0];
    pcl_point.y = X[1];
    pcl_point.z = X[2];
  }

  // Target position based on haz cam measurements
  Eigen::Vector3d intersection_pcl;
  if (pano_view::intersectRayPointCloud(pcl_cloud,
                                        msg_conversions::ros_pose_to_eigen_transform(ground_truth).translation(),
                                        vector, intersection_pcl)) {
    std::cout << "Intersection point pcl: (" << intersection_pcl.x() << ", " << intersection_pcl.y() << ", "
              << intersection_pcl.z() << ")" << std::endl;
  } else {
    std::cout << "No pcl intersection found." << std::endl;
  }

  // Target position based on 3D mesh model
  Eigen::Vector3d intersection_mesh;
  if (FLAGS_mesh_name != "") {
    if (pano_view::intersectRayMesh(FLAGS_mesh_name,
                                    msg_conversions::ros_pose_to_eigen_transform(ground_truth).translation(), vector,
                                    intersection_mesh)) {
      std::cout << "Intersection point pcl: (" << intersection_mesh.x() << ", " << intersection_mesh.y() << ", "
                << intersection_mesh.z() << ")" << std::endl;
    } else {
      std::cout << "No mesh intersection found." << std::endl;
    }
  }

  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
