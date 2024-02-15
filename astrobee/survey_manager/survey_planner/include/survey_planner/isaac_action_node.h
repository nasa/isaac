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

#ifndef SURVEY_PLANNER_ISAAC_ACTION_NODE_H_
#define SURVEY_PLANNER_ISAAC_ACTION_NODE_H_

#include <plansys2_executor/ActionExecutorClient.hpp>

#include <string>

namespace plansys2_actions {

class IsaacAction : public plansys2::ActionExecutorClient {
 public:
  IsaacAction(ros::NodeHandle nh, const std::string& action, const std::chrono::nanoseconds& rate);
  ~IsaacAction();

 protected:
  void do_work();

  float progress_;
  std::string robot_name_, action_name_;
  int pid_;
  std::string command_;
};
}  // namespace plansys2_actions


// Main entry point for application
int isaac_action_main(int argc, char *argv[], const char* action_name);

#endif  // SURVEY_PLANNER_ISAAC_ACTION_NODE_H_
