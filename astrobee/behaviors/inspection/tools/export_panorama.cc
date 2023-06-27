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

// Command line flags (not used yet)
#include <gflags/gflags.h>
#include <gflags/gflags_completions.h>

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// FSW includes
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>
#include <ff_common/ff_names.h>
#include <ff_util/ff_action.h>
// #include <isaac_msgs/AckS.h>
#include <geometry_msgs/PoseArray.h>

// C++ STL includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <cmath>

#define DEG2RAD 3.1415/180.0

// Include inspection lib
#include "inspection/inspection.h"

// Robot namespace
DEFINE_string(ns, "", "Robot namespace");

// Configurable Parameters
DEFINE_string(camera, "sci_cam", "Camera to use");
DEFINE_string(panorama_mode, "", "Panorama configuration pre-set");
DEFINE_double(pan_max, 180.0, "Panorama: maximum pan");
DEFINE_double(pan_min, -180.0, "Panorama: minimum pan");
DEFINE_double(tilt_max, 90.0, "Panorama: maximum tilt");
DEFINE_double(tilt_min, -90.0, "Panorama: minimum tilt");
DEFINE_double(h_fov, -1.0, "Panorama: camera horizontal fov, default -1 uses camera matrix");
DEFINE_double(v_fov, -1.0, "Panorama: camera vertical fov, default -1 uses camera matrix");
DEFINE_double(overlap, 0.5, "Panorama: overlap between images");
DEFINE_double(att_tol, 5.0, "Panorama: attitude tolerance due to mobility");

// Plan files
DEFINE_string(panorama_poses, "/resources/scicam_panorama.txt", "Panorama poses list to map");
DEFINE_string(panorama_out, "/resources/pano_out.txt", "Panorama poses output");


bool has_only_whitespace_or_comments(const std::string & str) {
  for (std::string::const_iterator it = str.begin(); it != str.end(); it++) {
    if (*it == '#') return true;  // No need to check further
    if (*it != ' ' && *it != '\t' && *it != '\n' && *it != '\r') return false;
  }
  return true;
}

// Read inspection poses from given files
bool ReadPanoramaConfig(double* pan_radius_degrees, double* tilt_rad_deg, double* h_fov_deg, double* v_fov_deg,
                        double* overlap, double* plan_att_tol_deg) {
  std::ifstream ifs(std::string(ros::package::getPath("inspection") + "/resources/pano_test_cases.csv").c_str());

  // Check if file exists
  if (!ifs.is_open()) {
    std::cout << "Could not open file: " << ros::package::getPath("inspection") + "/resources/pano_test_cases.csv"
              << std::endl;
    return false;
  }

  std::string line;
  std::string label;
  double test_att_tol_deg;
  while (getline(ifs, line)) {
    if (has_only_whitespace_or_comments(line)) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    line.erase(std::remove(line.begin(), line.end(), '"'), line.end());

    std::istringstream is(line);
    if ((is >> label >> *pan_radius_degrees >> *tilt_rad_deg >> *h_fov_deg >> *v_fov_deg >> *overlap >>
         *plan_att_tol_deg >> test_att_tol_deg)) {
      if (FLAGS_panorama_mode == label) {
        return true;
      }
    } else {
      std::cout << "Ignoring invalid line: " << line  << std::endl;
      continue;
    }
  }
  std::cout << "Could not find panorama_mode specified"  << std::endl;
  return false;
}

void ReadFile(std::string file, isaac_msgs::InspectionGoal &goal) {
    geometry_msgs::Pose pose;
    goal.inspect_poses.header.frame_id = FLAGS_camera;
    // Read file
    std::ifstream ifs((file).c_str());
    if (!ifs.is_open()) {
      std::cout << "Could not open file: " << file << std::endl;
      return;
    }
    std::string line;
    tf2::Quaternion quat_robot;
    while (getline(ifs, line)) {
      if (has_only_whitespace_or_comments(line)) continue;

      std::istringstream is(line);
      double origin_x, origin_y, origin_z;
      double euler_roll, euler_pitch, euler_yaw;
      double quat_x, quat_y, quat_z, quat_w;
      if ((is >> origin_x >> origin_y >> origin_z >> quat_x >> quat_y >> quat_z >> quat_w)) {
          // Position
          pose.position.x = origin_x;
          pose.position.y = origin_y;
          pose.position.z = origin_z;

          // Orientation
          pose.orientation.x = quat_x;
          pose.orientation.y = quat_y;
          pose.orientation.z = quat_z;
          pose.orientation.w = quat_w;
          goal.inspect_poses.poses.push_back(pose);

      } else {
        std::istringstream is(line);
        if ((is >> origin_x >> origin_y >> origin_z >> euler_roll >> euler_pitch >> euler_yaw)) {
          // Position
          pose.position.x = origin_x;
          pose.position.y = origin_y;
          pose.position.z = origin_z;

          quat_robot.setRPY(euler_roll * DEG2RAD,
                            euler_pitch * DEG2RAD,
                            euler_yaw * DEG2RAD);
          // Orientation
          pose.orientation.x = quat_robot.x();
          pose.orientation.y = quat_robot.y();
          pose.orientation.z = quat_robot.z();
          pose.orientation.w = quat_robot.w();
          goal.inspect_poses.poses.push_back(pose);

        } else {
          std::cout << "Ignoring invalid line: " << line  << std::endl;
          continue;
        }
      }
    }
}
  // Callback to handle reconfiguration requests
  bool ReconfigureCallback(dynamic_reconfigure::Config & config) {
    // if (cfg_.Reconfigure(config)) {
    //   // inspection_->ReadParam();
    //   return true;
    // }
    return false;
  }


int main(int argc, char *argv[]) {
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun inspection export_panorama <opts>. ");
  google::SetVersionString("0.1.0");
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Initialize a ros node
  ros::init(argc, argv, "panorama_export", ros::init_options::AnonymousName);

  // Create a node handle
  std::string ns = std::string("/") + FLAGS_ns;
  ros::NodeHandle nh;

  // Set the config path to ISAAC
  ff_util::ConfigServer cfg_;
  char *path = getenv("CUSTOM_CONFIG_DIR");
  if (path != NULL)
    cfg_.SetPath(path);
  // Grab some configuration parameters for this node from the LUA config reader
  cfg_.Initialize(&nh, "behaviors/inspection.config");
  if (!cfg_.Listen(boost::bind(&ReconfigureCallback, _1)))
    return 0;

  if (FLAGS_panorama_mode == "") {
    // Set parameters from cmd line
    cfg_.Set<double>("h_fov", FLAGS_h_fov);
    cfg_.Set<double>("v_fov", FLAGS_v_fov);

    cfg_.Set<double>("pan_min", FLAGS_pan_min);
    cfg_.Set<double>("pan_max", FLAGS_pan_max);
    cfg_.Set<double>("tilt_min", FLAGS_tilt_min);
    cfg_.Set<double>("tilt_max", FLAGS_tilt_max);
    cfg_.Set<double>("overlap", FLAGS_overlap);
    cfg_.Set<double>("att_tol", FLAGS_att_tol);
  } else {
      // Read file panorama config
      double pan_radius_degrees, tilt_rad_deg, h_fov_deg, v_fov_deg, overlap, plan_att_tol_deg;
      if (ReadPanoramaConfig(&pan_radius_degrees, &tilt_rad_deg, &h_fov_deg, &v_fov_deg, &overlap, &plan_att_tol_deg)) {
        cfg_.Set<double>("h_fov", h_fov_deg);
        cfg_.Set<double>("v_fov", v_fov_deg);

        cfg_.Set<double>("pan_min", -pan_radius_degrees);
        cfg_.Set<double>("pan_max",  pan_radius_degrees);
        cfg_.Set<double>("tilt_min", -tilt_rad_deg);
        cfg_.Set<double>("tilt_max",  tilt_rad_deg);
        cfg_.Set<double>("overlap", overlap);
        cfg_.Set<double>("att_tol", plan_att_tol_deg);
      }
    }

  // Initiate inspection library
  inspection::Inspection inspection_(&nh, &cfg_);
  isaac_msgs::InspectionGoal goal;

  // Read file
  // std::cout << "Reading: " << FLAGS_panorama_poses << std::endl;
  std::string path_inspection = std::string(ros::package::getPath("inspection"));
  ReadFile(path_inspection + FLAGS_panorama_poses, goal);

  // ROS_ERROR("Generate Panorama");
  inspection_.GeneratePanoramaSurvey(goal.inspect_poses);

  // Write in file
  std::ofstream myfile;
  std::string path_output = ros::package::getPath("inspection") + FLAGS_panorama_out;
  myfile.open(path_output);
  for (int i = 0; i < goal.inspect_poses.poses.size(); i++) {
    tf2::Quaternion quat(goal.inspect_poses.poses[i].orientation.x, goal.inspect_poses.poses[i].orientation.y,
        goal.inspect_poses.poses[i].orientation.z, goal.inspect_poses.poses[i].orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    myfile << goal.inspect_poses.poses[i].position.x << " " << goal.inspect_poses.poses[i].position.y << " "
           << goal.inspect_poses.poses[i].position.z << " " << round(roll / (DEG2RAD)*10.0) / 10.0 << " "
           << round(pitch / (DEG2RAD)*10.0) / 10.0 << " " << round(yaw / (DEG2RAD)*10.0) / 10.0 << "\n";
  }
  myfile.close();


  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}

