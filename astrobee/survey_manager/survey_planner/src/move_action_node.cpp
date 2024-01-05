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

#include <string>

namespace plansys2_actions {

class MoveAction : public plansys2::ActionExecutorClient {
 public:
  MoveAction(ros::NodeHandle nh, const std::string& action, const std::chrono::nanoseconds& rate)
      : ActionExecutorClient(nh, action, rate) {}

 protected:
  void do_work() {
    ROS_ERROR_STREAM("Executing [MOVE]");
    // std::string from, towards;
    // from = get_arguments()[1];
    // towards = get_arguments()[2];
  }
};
}  // namespace plansys2_actions


// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "move_action");

  std::string name = ros::this_node::getName();
  if (name.empty() || (name.size() == 1 && name[0] == '/'))
    name = "default";
  else if (name[0] == '/')
    name = name.substr(1);

  ros::NodeHandle nh("~");
  nh.setParam("action_name", "move");

  // Start action node
  // We could actually add multiple action nodes here being aware that we might need a ros::AsyncSpinner
  // (https://github.com/Bckempa/ros2_planning_system/blob/noetic-devel/plansys2_bt_actions/src/bt_action_node.cpp#L41)
  auto action_node = std::make_shared<plansys2_actions::MoveAction>(nh, "move",  std::chrono::seconds(1));
  action_node->trigger_transition(ros::lifecycle::CONFIGURE);

  ROS_ERROR_STREAM("Starting action");

  // Synchronous mode
  ros::spin();
  // Make for great success
  return 0;
}
