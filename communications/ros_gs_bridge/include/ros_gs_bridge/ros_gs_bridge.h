/* Copyright (c) 2021, United States Government, as represented by the"
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

#ifndef ROS_GS_BRIDGE_ROS_GS_BRIDGE_H_
#define ROS_GS_BRIDGE_ROS_GS_BRIDGE_H_

// Standard ROS includes
#include <ros/ros.h>

// Shared project includes
#include <ff_util/ff_action.h>
#include <ff_util/ff_names.h>
#include <isaac_util/isaac_names.h>

// Messages
#include <ff_msgs/AccessControlStateStamped.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/GuestScienceData.h>
#include <isaac_hw_msgs/WifiSignals.h>

// Actions
#include <isaac_msgs/ImageInspectionAction.h>
#include <isaac_msgs/InspectionAction.h>

// Services
#include <std_srvs/Empty.h>

// C++ headers
#include <string>
#include <queue>

namespace ros_gs_bridge {

class RosGsBridge{
 public:
  RosGsBridge();
  ~RosGsBridge();

  bool Initialize(ros::NodeHandle *nh);

  // Message Callbacks
  void AccessControlStateCallback(ff_msgs::AccessControlStateStampedConstPtr
                                                                  const& state);
  void GuestScienceDataCallback(ff_msgs::GuestScienceDataConstPtr const& data);


  // Service Callbacks
  bool GrabControlService(std_srvs::Empty::Request& req,
                          std_srvs::Empty::Response& res);

  // Action Callbacks
  // TODO(Katie) Add image inspection feedback for completeness
  void ImageInspectionResultCallback(
                      ff_util::FreeFlyerActionState::Enum const& state,
                      isaac_msgs::ImageInspectionResultConstPtr const& result);

  void InspectionGoalCallback(isaac_msgs::InspectionGoalConstPtr const& goal);
  void InspectionPreemptCallback();
  void InspectionCancelCallback();

  void FillTimeIdInGSCmd();
  void GrabControl(std::string cookie);
  bool HasControl();
  void RequestControl();
  void SendNextGSCommand();

 private:
  bool requesting_control_, grabbing_control_, send_command_;

  ff_util::FreeFlyerActionClient<isaac_msgs::ImageInspectionAction> img_ac_;
  ff_util::FreeFlyerActionServer<isaac_msgs::InspectionAction> inspection_as_;

  int pub_queue_size_, sub_queue_size_, cmd_counter_;

  ros::NodeHandle* nh_;
  ros::Publisher cmd_pub_, wifi_signals_pub_;
  ros::ServiceServer grab_control_srv_;
  ros::Subscriber sub_access_control_state_, sub_gs_data_;

  // TODO(Katie): Make a queue of a new class you create. Class will need to
  // keep track of the custom guest science command associated with one type
  // of goal, feedback, result, or topic
  ff_msgs::CommandStamped custom_gs_command_;
  std::string controller_, isaac_apk_name_, isaac_controller_name_;
};

}  // namespace ros_gs_bridge

#endif  // ROS_GS_BRIDGE_ROS_GS_BRIDGE_H_
