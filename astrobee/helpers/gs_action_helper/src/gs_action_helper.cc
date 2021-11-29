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

#include "gs_action_helper/gs_action_helper.h"

namespace gs_action_helper {

GsActionHelper::GsActionHelper() :
  ff_util::FreeFlyerNodelet(NODE_GS_ACTION_HELPER, false),
  pub_queue_size_(10),
  sub_queue_size_(10) {
}

GsActionHelper::~GsActionHelper() {
}

void GsActionHelper::Initialize(ros::NodeHandle *nh) {
  nh_ = nh;

  // initialize actions
  img_as_.SetGoalCallback(
                        std::bind(&GsActionHelper::ImageInspectionGoalCallback,
                                  this,
                                  std::placeholders::_1));
  img_as_.SetPreemptCallback(
                      std::bind(&GsActionHelper::ImageInspectionPreemptCallback,
                                this));
  img_as_.SetCancelCallback(
                      std::bind(&GsActionHelper::ImageInspectionCancelCallback,
                                this));
  img_as_.Create(nh, ACTION_ANOMALY_IMG_ANALYSIS);


  inspection_ac_.SetFeedbackCallback(
                          std::bind(&GsActionHelper::InspectionFeedbackCallback,
                                    this,
                                    std::placeholders::_1));
  inspection_ac_.SetResultCallback(
                          std::bind(&GsActionHelper::InspectionResultCallback,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));
  inspection_ac_.Create(nh, ACTION_BEHAVIORS_INSPECTION);

  // initialize pubs
  pub_image_inspection_goal_ = nh_->advertise<isaac_msgs::ImageInspectionGoal>(
                                      TOPIC_GUEST_SCIENCE_IMAGE_INSPECTION_GOAL,
                                      pub_queue_size_,
                                      true);

  pub_inspection_feedback_ = nh_->advertise<isaac_msgs::InspectionFeedback>(
                                        TOPIC_GUEST_SCIENCE_INSPECTION_FEEDBACK,
                                        pub_queue_size_,
                                        true);

  pub_inspection_result_ = nh_->advertise<isaac_msgs::InspectionResult>(
                                          TOPIC_GUEST_SCIENCE_INSPECTION_RESULT,
                                          pub_queue_size_,
                                          true);

  // initialize subs
  sub_image_inspection_result_ = nh_->subscribe(
                                TOPIC_GUEST_SCIENCE_IMAGE_INSPECTION_RESULT,
                                sub_queue_size_,
                                &GsActionHelper::ImageInspectionResultCallback,
                                this);

  sub_inspection_goal_ = nh_->subscribe(TOPIC_GUEST_SCIENCE_INSPECTION_GOAL,
                                        sub_queue_size_,
                                        &GsActionHelper::InspectionGoalCallback,
                                        this);
}

// Message Callbacks
void GsActionHelper::ImageInspectionResultCallback(
                      isaac_msgs::ImageInspectionResultConstPtr const& state) {
  if (state->response > 0) {
    img_as_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, *state);
  } else if (state->response < 0) {
    img_as_.SendResult(ff_util::FreeFlyerActionState::ABORTED, *state);
  } else {
    img_as_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, *state);
  }
}

void GsActionHelper::InspectionGoalCallback(
                              isaac_msgs::InspectionGoalConstPtr const& goal) {
  inspection_ac_.SendGoal(*goal);
}

// Action callbacks
void GsActionHelper::ImageInspectionGoalCallback(
                          isaac_msgs::ImageInspectionGoalConstPtr const& goal) {
  pub_image_inspection_goal_.publish(goal);
}

void GsActionHelper::ImageInspectionPreemptCallback() {
  ROS_ERROR("gs action helper: Image inspection preempt not implemented yet!");
}

void GsActionHelper::ImageInspectionCancelCallback() {
  ROS_ERROR("gs action helper: Image inspection canceled not implemented yet!");
}

void GsActionHelper::InspectionFeedbackCallback(
                      isaac_msgs::InspectionFeedbackConstPtr const& feedback) {
  pub_inspection_feedback_.publish(feedback);
}

void GsActionHelper::InspectionResultCallback(
                          ff_util::FreeFlyerActionState::Enum const& state,
                          isaac_msgs::InspectionResultConstPtr const& result) {
  pub_inspection_result_.publish(result);
}

PLUGINLIB_EXPORT_CLASS(gs_action_helper::GsActionHelper, nodelet::Nodelet);

}  // namespace gs_action_helper
