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

#ifndef GS_ACTION_HELPER_GS_ACTION_HELPER_H_
#define GS_ACTION_HELPER_GS_ACTION_HELPER_H_

// Standard ROS includes
#include <ros/ros.h>

// Shared project includes
#include <ff_util/ff_action.h>
#include <ff_util/ff_nodelet.h>
#include <isaac_util/isaac_names.h>

// Actions
#include <isaac_msgs/ImageInspectionAction.h>
#include <isaac_msgs/InspectionAction.h>

// C++ headers
#include <string>

namespace gs_action_helper {

class GsActionHelper : public ff_util::FreeFlyerNodelet {
 public:
  GsActionHelper();
  ~GsActionHelper();

  void Initialize(ros::NodeHandle *nh);

  // Message Callbacks
  void ImageInspectionResultCallback(isaac_msgs::ImageInspectionResultConstPtr
                                                                const& result);
  void InspectionGoalCallback(isaac_msgs::InspectionGoalConstPtr const& goal);

  // Action Callbacks
  void ImageInspectionGoalCallback(isaac_msgs::ImageInspectionGoalConstPtr
                                                                  const& goal);
  void ImageInspectionPreemptCallback();
  void ImageInspectionCancelCallback();

  void InspectionFeedbackCallback(isaac_msgs::InspectionFeedbackConstPtr
                                                              const& feedback);
  void InspectionResultCallback(
                      ff_util::FreeFlyerActionState::Enum const& state,
                      isaac_msgs::InspectionResultConstPtr const& result);

 private:
  ff_util::FreeFlyerActionServer<isaac_msgs::ImageInspectionAction> img_as_;
  ff_util::FreeFlyerActionClient<isaac_msgs::InspectionAction> inspection_ac_;

  int pub_queue_size_, sub_queue_size_;

  ros::NodeHandle* nh_;
  ros::Publisher pub_image_inspection_goal_, pub_inspection_feedback_;
  ros::Publisher pub_inspection_result_;
  ros::Subscriber sub_image_inspection_result_, sub_inspection_goal_;
};

}  // namespace gs_action_helper

#endif  // GS_ACTION_HELPER_GS_ACTION_HELPER_H_
