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
  std::stringstream ss(input_json);
  std::string token;


  // Convert the output list to JSON
  std::stringstream outputJson;
  outputJson << '[';

  while (std::getline(ss, token, '}')) {
    if (token.empty())
      continue;

    token += '}';
    std::stringstream elementStream(token);

    std::string elementToken;
    // std::getline(elementStream, elementToken, '{'); // Skip the opening brace

    // // Extract the values for point1
    // std::getline(elementStream, elementToken, ',');
    // std::stringstream point1Stream(elementToken);
    // std::getline(point1Stream, elementToken, ',');
    // element.point1.x = std::stof(elementToken);
    // std::getline(point1Stream, elementToken, ',');
    // element.point1.y = std::stof(elementToken);
    // std::getline(point1Stream, elementToken, ',');
    // element.point1.z = std::stof(elementToken);

    // // Extract the values for point2
    // std::getline(elementStream, elementToken, ',');
    // std::stringstream point2Stream(elementToken);
    // std::getline(point2Stream, elementToken, ',');
    // element.point2.x = std::stof(elementToken);
    // std::getline(point2Stream, elementToken, ',');
    // element.point2.y = std::stof(elementToken);
    // std::getline(point2Stream, elementToken, ',');
    // element.point2.z = std::stof(elementToken);

    // camera.SetTransform((msg_conversions::ros_pose_to_eigen_transform(ground_truth) *
    // transform_body_to_cam).inverse());

    // if (camera.GetCamXYFromPoint()) {

    // }

    // // Convert the values of point1 to a JSON string
    // std::stringstream point1Json;
    // point1Json << "{\"x\":" << element.point1.x << ",\"y\":" << element.point1.y << ",\"z\":" << element.point1.z <<
    // "}";

    // // Convert the values of point2 to a JSON string
    // std::stringstream point2Json;
    // point2Json << "{\"x\":" << element.point2.x << ",\"y\":" << element.point2.y << ",\"z\":" << element.point2.z <<
    // "}";

    // // Convert the values of attitude to a JSON string
    // std::stringstream attitudeJson;
    // attitudeJson << "{\"w\":" << element.attitude.w << ",\"x\":" << element.attitude.x << ",\"y\":" <<
    // element.attitude.y << ",\"z\":" << element.attitude.z << "}";

    // // Combine the JSON strings of point1, point2, and attitude into a single JSON string for the current element
    // outputJson << "{\"point1\":" << point1Json.str() << ",\"point2\":" << point2Json.str() << ",\"attitude\":" <<
    // attitudeJson.str() << "}";

    // if (i != outputList.size() - 1) {
    //     outputJson << ',';
    // }
  }
  outputJson << ']';

  // Print the output JSON
  std::cout << outputJson.str() << std::endl;




  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
