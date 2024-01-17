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
#include <iostream>

namespace plansys2_actions {

class UndockAction : public plansys2::ActionExecutorClient {
 public:
  UndockAction(ros::NodeHandle nh, const std::string& action, const std::chrono::nanoseconds& rate)
      : ActionExecutorClient(nh, action, rate) {
    progress_ = 0.0;
    process_pipe_ = nullptr;
  }

 protected:
  void do_work() {
    std::string from, towards;

    if (get_arguments().size() > 2) {
      robot_name_ = get_arguments()[0];
      from = get_arguments()[1];
      towards = get_arguments()[2];
    } else {
      finish(false, 1.0, "Not enough arguments for [MOVE] command");
    }

    // Start process if not started yet
    if (progress_ == 0.0) {
      std::string command =
        "rosrun survey_planner command_astrobee " + robot_name_ + " undock " + towards + " " + from + " run1";

      std::cout << command << std::endl;
      // Open a pipe to a command and get a FILE* for reading
      process_pipe_ = popen(command.c_str(), "r");

      if (!process_pipe_) {
        perror("popen");
        finish(false, 1.0, "Failed to start the process");
        return;
      }

      // Get the process ID
      progress_ = 0.02;
      return;
    }

    char buffer[1028];

    if (fgets(buffer, 1028, process_pipe_) != NULL) {
      std::cout << buffer << std::endl;
      if (progress_ < 1.0) {
        progress_ += 0.02;
        send_feedback(progress_, "Undock running");
      }

      std::cout << "\t ** [Undock] Robot " << robot_name_ << " moving from " << from << " towards " << towards
                << " ... [" << std::min(100.0, progress_ * 100.0) << "%]  " << std::endl;
    } else {
      int status = pclose(process_pipe_);
      std::cout << "Finished.\n" << std::endl;
      if (status == 0) {
        std::cout << "Command exited with status success " << std::endl;
        finish(true, 1.0, "Undock completed");
      } else {
        std::cout << "Command terminated with status fail:  " << status << std::endl;
        finish(false, 1.0, "Undock terminated by signal");
      }
    }
  }

  float progress_;
  std::string robot_name_;
  FILE* process_pipe_;
};
}  // namespace plansys2_actions


// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "undock_action");

  std::string name = ros::this_node::getName();
  if (name.empty() || (name.size() == 1 && name[0] == '/'))
    name = "default";
  else if (name[0] == '/')
    name = name.substr(1);

  ros::NodeHandle nh("~");
  nh.setParam("action_name", "undock");

  // Start action node
  // We could actually add multiple action nodes here being aware that we might need a ros::AsyncSpinner
  // (https://github.com/Bckempa/ros2_planning_system/blob/noetic-devel/plansys2_bt_actions/src/bt_action_node.cpp#L41)
  auto action_node = std::make_shared<plansys2_actions::UndockAction>(nh, "undock",  std::chrono::seconds(1));
  action_node->trigger_transition(ros::lifecycle::CONFIGURE);

  // Synchronous mode
  ros::spin();
  // Make for great success
  return 0;
}
