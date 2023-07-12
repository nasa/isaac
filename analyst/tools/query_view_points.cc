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

Eigen::Vector3d read_vector(Json::Value element) {
  Eigen::Vector3d vector_out;
  vector_out[0] = element[0].asFloat();
  vector_out[1] = element[1].asFloat();
  vector_out[2] = element[2].asFloat();
  return vector_out;
}

Eigen::Vector2d rescale_image(Eigen::Vector2d cam_size, int x, int y, int width, int height) {
  Eigen::Vector2d vector_out;
  vector_out[0] = x * width  / cam_size[0];
  vector_out[1] = y * height / cam_size[1];
  return vector_out;
}
Json::Value vector_2d_to_json(Eigen::Vector2d coord) {
  Json::Value coord_json;
  coord_json[0] = coord[0];
  coord_json[1] = coord[1];
  return coord_json;
}

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
  Json::Value output_json;
  // Process each list element
  for (const auto& element : json) {
    // Assuming each list element has two dictionaries
    if (element.size() != 13) {
      std::cerr << "Invalid list element encountered. Haz size " << element.size() << std::endl;
      continue;
    }

    geometry_msgs::Pose ground_truth;
    ground_truth.position.x = element[1].asFloat();
    ground_truth.position.y = element[2].asFloat();
    ground_truth.position.z = element[3].asFloat();
    ground_truth.orientation.x = element[4].asFloat();
    ground_truth.orientation.y = element[5].asFloat();
    ground_truth.orientation.z = element[6].asFloat();
    ground_truth.orientation.w = element[7].asFloat();
    Eigen::Vector3d target = read_vector(element[8]);
    Eigen::Vector3d c1 = read_vector(element[9]);
    Eigen::Vector3d c2 = read_vector(element[10]);
    Eigen::Vector3d c3 = read_vector(element[11]);
    Eigen::Vector3d c4 = read_vector(element[12]);

    int x, y;
    int height = 4008;
    int width = 5344;
    int c1_x, c1_y, c2_x, c2_y, c3_x, c3_y, c4_x, c4_y;
    if (camera.GetCamXYFromPoint(msg_conversions::ros_pose_to_eigen_transform(ground_truth), target, x, y)) {
      Json::Value output_element_json;
      output_element_json["img"] = element[0];

      Eigen::Vector2d cam_size = (cam_params.GetDistortedSize()).cast<double>();

      Eigen::Vector2d coord = rescale_image(cam_size, x, y, width, height);
      output_element_json["coord"] = vector_2d_to_json(coord);

      if (!camera.GetCamXYFromPoint(msg_conversions::ros_pose_to_eigen_transform(ground_truth), c1, c1_x, c1_y)) {
        continue;
      }
      Eigen::Vector2d coord_c1 = rescale_image(cam_size, c1_x, c1_y, width, height);
      output_element_json["coord_c1"] = vector_2d_to_json(coord_c1);

      if (!camera.GetCamXYFromPoint(msg_conversions::ros_pose_to_eigen_transform(ground_truth), c2, c2_x, c2_y)) {
        continue;
      }
      Eigen::Vector2d coord_c2 = rescale_image(cam_size, c2_x, c2_y, width, height);
      output_element_json["coord_c2"] = vector_2d_to_json(coord_c2);

      if (!camera.GetCamXYFromPoint(msg_conversions::ros_pose_to_eigen_transform(ground_truth), c3, c3_x, c3_y)) {
        continue;
      }
      Eigen::Vector2d coord_c3 = rescale_image(cam_size, c3_x, c3_y, width, height);
      output_element_json["coord_c3"] = vector_2d_to_json(coord_c3);

      if (!camera.GetCamXYFromPoint(msg_conversions::ros_pose_to_eigen_transform(ground_truth), c4, c4_x, c4_y)) {
        continue;
      }
      Eigen::Vector2d coord_c4 = rescale_image(cam_size, c4_x, c4_y, width, height);
      output_element_json["coord_c4"] = vector_2d_to_json(coord_c4);

      output_json.append(output_element_json);
    }
  }

  // Print the output JSON
  std::cout << output_json << std::endl;

  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
