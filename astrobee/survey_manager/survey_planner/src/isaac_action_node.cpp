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

#include <plansys2_executor/ActionExecutorClient.hpp>

#include <sys/types.h>
#include <sys/wait.h>

#include <algorithm>
#include <string>
#include <vector>
#include <iostream>

#include "survey_planner/isaac_action_node.h"

namespace plansys2_actions {

IsaacAction::IsaacAction(ros::NodeHandle nh, const std::string& action, const std::chrono::nanoseconds& rate)
    : ActionExecutorClient(nh, action, rate) {
  action_name_ = action;
  progress_ = 0.0;
  pid_ = 0;
  command_ = "";
}

void IsaacAction::do_work() {
  std::string from, towards;

  const std::vector<std::string>& command_args = get_arguments();
  if (command_args.size() < 3) {
    finish(false, 1.0, "Not enough arguments for [MOVE] command");
  }

  // Start process if not started yet
  if (progress_ == 0.0) {
    pid_ = fork();
    if (pid_ < 0) {
      perror("Fork failed.");
      finish(false, 1.0, "Failed to start the process");
    } else if (pid_ == 0) {
      const char* args[4];
      args[0] = "sh";
      args[1] = "-c";
      command_ = "rosrun survey_planner command_astrobee ";
      command_ += action_name_ + " ";
      for (unsigned int i = 0; i < command_args.size(); i++) {
          command_ += command_args[i] + " ";
      }
      args[2] = command_.c_str();
      args[3] = NULL;
      printf("%s\n", args[2]);
      execvpe("sh", (char* const*)args, environ);
      perror("Failed to execute command.");
      printf("EXITING FAILURE %d\n", getpid());
      exit(-1);
    } else {
      progress_ = 0.02;
      return;
    }
  }

  if (progress_ < 1.0) {
    progress_ += 0.02;
    send_feedback(progress_, action_name_ + " running");
  }

  std::cout << "\t ** [" << action_name_ << "] " << command_ << " [" << std::min(100.0, progress_ * 100.0) << "%]  "
            << std::endl;
  int status;
  int result = waitpid(-1, &status, WNOHANG);
  printf("Result: %d %d %d\n", result, pid_, status);
  if (result < 0) {
    perror("Failed to wait for pid.");
    progress_ = 0.0;
    finish(false, 1.0, "Unexpected error waiting for process.");
  } else if (result == pid_) {
    if (status == 0) {
      std::cout << "Command exited with status success " << std::endl;
      progress_ = 0.0;
      finish(true, 1.0, action_name_ + " completed");
    } else {
      std::cout << "Command terminated with status fail:  " << status << std::endl;
      progress_ = 0.0;
      finish(false, 1.0, action_name_ + " terminated by signal");
    }
  }
}
}  // namespace plansys2_actions


// Main entry point for application
int isaac_action_main(int argc, char *argv[], const char* action_name) {
  // Initialize a ros node
  ros::init(argc, argv, (std::string(action_name) + "_action").c_str());

  std::string name = ros::this_node::getName();
  if (name.empty() || (name.size() == 1 && name[0] == '/'))
    name = "default";
  else if (name[0] == '/')
    name = name.substr(1);

  ros::NodeHandle nh("~");
  nh.setParam("action_name", action_name);

  // Start action node
  // We could actually add multiple action nodes here being aware that we might need a ros::AsyncSpinner
  // (https://github.com/Bckempa/ros2_planning_system/blob/noetic-devel/plansys2_bt_actions/src/bt_action_node.cpp#L41)
  auto action_node = std::make_shared<plansys2_actions::IsaacAction>(nh, action_name,  std::chrono::seconds(1));
  action_node->trigger_transition(ros::lifecycle::CONFIGURE);

  // Synchronous mode
  ros::spin();
  // Make for great success
  return 0;
}
