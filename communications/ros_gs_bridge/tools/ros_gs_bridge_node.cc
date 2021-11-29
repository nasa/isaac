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

#include "ros_gs_bridge/ros_gs_bridge.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_gs_bridge");

  // ros::NodeHandle nh("bumble");
  ros::NodeHandle nh;
  ros_gs_bridge::RosGsBridge rgb;
  if (rgb.Initialize(&nh)) {
    ros::spin();
  }

  return 0;
}
