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

// FSW includes
#include <config_reader/config_reader.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_action.h>
#include <ff_msgs/AckStamped.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <std_msgs/String.h>

// C++ STL includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <cmath>

// Robot namespace
DEFINE_string(ns, "", "Robot namespace");

void publish_cmd(ros::NodeHandle & nh, ros::Publisher & cmd_pub,
                 std::string const& cmdName, std::string const& cmdVal) {
  ff_msgs::CommandArg arg;
  std::vector<ff_msgs::CommandArg> cmd_args;

  // The command sends two strings. The first has the app name,
  // and the second the command value, encoded as json.

  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = "gov.nasa.arc.irg.astrobee.sci_cam_image";
  cmd_args.push_back(arg);

  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = "{\"name\": \"" + cmdName + "\"";
  if (cmdVal != "")
    arg.s += ", " + cmdVal;
  arg.s += "}";
  cmd_args.push_back(arg);

  std::cout << "JSON string: " << arg.s << std::endl;

  ff_msgs::CommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE;
  cmd.cmd_id = "inspection" + std::to_string(ros::Time::now().toSec());
  cmd.cmd_src = "guest science";
  cmd.cmd_origin = "guest science";
  cmd.args = cmd_args;

  cmd_pub.publish(cmd);
}

int main(int argc, char *argv[]) {
  // Gather some data from the command
  google::SetUsageMessage("Usage: sci_cam_tool -cmd <cmd name>. "
                          "See below for allowed values for the -cmd option.");
  google::SetVersionString("0.1.0");
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Accepted commands
  std::vector<std::string> cmd_names, cmd_vals;
  cmd_names.push_back("takePicture");                     cmd_vals.push_back("");
  cmd_names.push_back("setAutoExposure");                 cmd_vals.push_back("\"auto\": true");
  cmd_names.push_back("setContinuousPictureTaking");      cmd_vals.push_back("\"continuous\": true");
  cmd_names.push_back("setFocusDistance");                cmd_vals.push_back("\"distance\": 0.39");
  cmd_names.push_back("setFocusMode");                    cmd_vals.push_back("\"mode\": \"manual\"");
  cmd_names.push_back("setPublishImage");                 cmd_vals.push_back("\"publish\": true");
  cmd_names.push_back("setPublishedImageSize");           cmd_vals.push_back("\"width\": 640, \"height\": 480");
  cmd_names.push_back("setPublishedImageType");           cmd_vals.push_back("\"type\": \"color\"");
  cmd_names.push_back("setSavePicturesToDisk");           cmd_vals.push_back("\"save\": true");

  // Initialize a ros node
  ros::init(argc, argv, "sci_cam_tool", ros::init_options::AnonymousName);

  // Create a node handle
  std::string ns = std::string("/") + FLAGS_ns;
  ros::NodeHandle nh;

  // Initialize the command publisher
  std::string topic = TOPIC_COMMAND;
  ros::Publisher cmd_pub = nh.advertise<ff_msgs::CommandStamped>(topic, 10);
  std::cout << "Publishing commands on topic: /" << topic << "\n";

  // A publisher must always use a loop. ROS does not handle
  // gracefully a publisher being declared, a command being published,
  // and it quitting right away.
  while (1) {
    // Use the value 0 to quit. That because any time a user specifies
    // some invalid input, C++ casts it to 0.

    std::cout << "Specify the number of the command to publish and hit 'enter'.\n";
    std::cout << "Alternatively, enter the full command (with a modified value if desired).\n";
    for (size_t it = 0; it < cmd_names.size(); it++)
      std::cout << it + 1 << ") " << cmd_names[it] << " " << cmd_vals[it] << std::endl;
    std::cout << "0) Quit this program." << std::endl;

    // Get input from the user
    std::vector<std::string> inputs;
    std::string line, val;
    std::getline(std::cin, line);
    std::istringstream iss(line);
    while (iss >> val) {
      inputs.push_back(val);
    }

    if (inputs.size() == 0) {
      std::cout << "Quitting." << std::endl;
      return 0;
    }

    // See if the user entered the index of a command or a full command
    std::string cmd_name = "", cmd_val = "";
    if (inputs.size() == 1 && inputs[0].size() == 1 &&
        '0' <= inputs[0][0] && inputs[0][0] <= '9') {
      size_t cmd_index = atoi(inputs[0].c_str());
      if (cmd_index == 0) {
        std::cout << "Quitting." << std::endl;
        return 0;
      }

      cmd_index--;

      if (cmd_index >= cmd_names.size()) {
        std::cout << "Invalid choice." << std::endl;
        continue;
      }

      cmd_name = cmd_names[cmd_index];
      cmd_val  = cmd_vals[cmd_index];
    } else {
      cmd_name = inputs[0];
      if (inputs.size() > 1)
        cmd_val  = inputs[1];
    }

    std::cout << "Sending command: " << cmd_name << ' ' << cmd_val << std::endl;
    publish_cmd(nh, cmd_pub, cmd_name, cmd_val);
    std::cout << "---\n\n";

    ros::spinOnce();
  }

  return 0;
}

