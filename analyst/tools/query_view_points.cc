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

// Import messages
#include <ff_msgs/EkfState.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

// Shared project includes
#include <config_reader/config_reader.h>
#include <camera/camera_params.h>
#include <inspection/camera_projection.h>

// json file reader lib
#include <jsoncpp/json/allocator.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/value.h>

#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

// Parameters
DEFINE_string(camera, "sci_cam", "Camera name.");

DEFINE_double(min_distance,         0.2, "Anomaly: minimum distance to target (m)");
DEFINE_double(max_distance,         0.7, "Anomaly: maximum distance to target (m)");

int main(int argc, char** argv) {
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun inspection query_view_points <opts>");
  google::SetVersionString("0.1.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Check if an input list was provided
  if (argc < 2) {
    std::cerr << "Error: No input list provided." << std::endl;
    return 1;
  }

  // Read transforms
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("transforms.config");
  if (!config.ReadFiles()) LOG(FATAL) << "Failed to read config files.";

  // Read Transform
  std::string transform_str;
  Eigen::Affine3d transform_nav_to_cam;
  transform_str = "nav_cam_to_" + FLAGS_camera + "_transform";
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

  // Initialize camera view
  Eigen::Affine3d transform_body_to_cam = transform_body_to_nav * transform_nav_to_cam;
  geometry_msgs::Transform::ConstPtr msg_pointer(
    new geometry_msgs::Transform(msg_conversions::eigen_transform_to_ros_transform(transform_body_to_cam)));

  camera::CameraParameters cam_params(&config, FLAGS_camera.c_str());
  inspection::CameraView camera(cam_params, 2.0, 0.19, msg_pointer);





  // Extract the input list from the command-line argument
  std::string input_json = argv[1];

  Json::Reader reader;
  Json::Value json;
  if (!reader.parse(input_json, json)) {
      std::cerr << "Failed to parse the input JSON string." << std::endl;
      return 1;
  }
  // Check if the root element is an array
  if (!json.isArray()) {
      std::cerr << "Input JSON does not represent an array." << std::endl;
      return 1;
  }
  std::stringstream output_json;
  // Process each list element
  for (const auto& element : json) {
    // Assuming each list element has two dictionaries
    if (element.size() != 2 || !element[0].isObject() || !element[1].isArray()) {
      std::cerr << "Invalid list element encountered." << std::endl;
      continue;
    }

    // // Extract values from the first dictionary
    // const Json::Value& dict1 = element[0];
    // // Access the values using the desired keys
    // int seq = dict1["seq"].asInt();
    // int secs = dict1["stamp"]["secs"].asInt();
    // int nsecs = dict1["stamp"]["nsecs"].asInt();
    // std::string frame_id = dict1["frame_id"].asString();

    // Extract values from the second dictionary (assuming it contains 7 float values)
    const Json::Value& dict = element[1];
    std::vector<float> floatValues;
    for (const auto& value : dict) {
      if (value.isConvertibleTo(Json::realValue))
        floatValues.push_back(value.asFloat());
    }

    Eigen::Vector3d target;
    target[0] = element[1][0].asFloat();
    target[1] = element[1][1].asFloat();
    target[2] = element[1][2].asFloat();
    geometry_msgs::Pose ground_truth;
    ground_truth.position.x = element[1][3].asFloat();
    ground_truth.position.y = element[1][4].asFloat();
    ground_truth.position.z = element[1][5].asFloat();
    ground_truth.orientation.x = element[1][6].asFloat();
    ground_truth.orientation.y = element[1][7].asFloat();
    ground_truth.orientation.z = element[1][8].asFloat();
    ground_truth.orientation.w = element[1][9].asFloat();

    int x, y;

    if (camera.GetCamXYFromPoint((msg_conversions::ros_pose_to_eigen_transform(ground_truth) *
    transform_body_to_cam).inverse(), target, x, y)) {
    output_json << element;
    }
  }

  // Print the output JSON
  std::cout << output_json.str() << std::endl;

  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
