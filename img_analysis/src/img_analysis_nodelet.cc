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

// Standard ROS includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// Shared project includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_fsm.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>
#include <isaac_util/isaac_names.h>

// Anomaly detectors
#include <img_analysis/img_vent.h>

// Software messages

// Actions
#include <isaac_msgs/ImageInspectionAction.h>

namespace img_analysis {

// Match the internal states and responses with the message definition
using FSM = ff_util::FSM;
using STATE = isaac_msgs::ImageInspectionFeedback;
using RESPONSE = isaac_msgs::ImageInspectionResult;

class ImageAnalysisNode : public ff_util::FreeFlyerNodelet {
 public:
  // All possible events that can occur
  enum : FSM::Event {
    IMAGE_WAIT       = (1<<0),    // Waiting for the image to come
    IMAGE_ANALYSED   = (1<<1),    // Image was analysed
  };

  ImageAnalysisNode() : ff_util::FreeFlyerNodelet(NODE_IMG_ANALYSIS, false),
    fsm_(STATE::WAITING, std::bind(&ImageAnalysisNode::UpdateCallback,
      this, std::placeholders::_1, std::placeholders::_2)) {
    // Add the state transition lambda functions - refer to the FSM diagram
    // [0]
    fsm_.Add(STATE::WAITING,
      IMAGE_WAIT, [this](FSM::Event const& event) -> FSM::State {
        ROS_ERROR_STREAM("CHANGE STATE TO ANALYSING");
        return STATE::ANALYSING;
      });
    // [1]
    fsm_.Add(STATE::ANALYSING,
      IMAGE_ANALYSED, [this](FSM::Event const& event) -> FSM::State {
        ROS_ERROR_STREAM("CHANGE STATE TO WAITING");
        return STATE::WAITING;
      });
  }

  ~ImageAnalysisNode() {}

 protected:
  void Initialize(ros::NodeHandle* nh) {
    // Set the config path to ISAAC
    char *path = getenv("CUSTOM_CONFIG_DIR");
    if (path != NULL)
      cfg_.SetPath(path);
    // Grab some configuration parameters for this node from the LUA config reader
    cfg_.Initialize(GetPrivateHandle(), "behaviors/inspection.config");
    if (!cfg_.Listen(boost::bind(
      &ImageAnalysisNode::ReconfigureCallback, this, _1)))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                         "Could not load config");


    // Initialize the command publisher
    cmd_pub_ = nh->advertise<ff_msgs::CommandStamped>(
                      TOPIC_COMMUNICATIONS_DDS_COMMAND, 10);
    sub_sci_cam_ = nh->subscribe(TOPIC_HARDWARE_SCI_CAM + std::string("/compressed"), 1,
                      &ImageAnalysisNode::SciCamCallback, this);

    // Setup the execute action
    server_.SetGoalCallback(std::bind(
      &ImageAnalysisNode::GoalCallback, this, std::placeholders::_1));
    server_.SetPreemptCallback(std::bind(
      &ImageAnalysisNode::PreemptCallback, this));
    server_.SetCancelCallback(std::bind(
      &ImageAnalysisNode::CancelCallback, this));
    server_.Create(nh, ACTION_ANOMALY_IMG_ANALYSIS);
  }

  void SciCamCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    // If the sci cam picture was not requested, ignore
    if (!sci_cam_req_)
      return;
    sci_cam_req_ = false;

    // Analyse the received picture
    switch (goal_.type) {
    // Vent command
    case isaac_msgs::ImageInspectionGoal::VENT:
    {
      // Analyse the received picture
      cv::Mat input_img;
      try {
          input_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      }
      catch (cv_bridge::Exception& e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }
      int classification = vent_analysis_.AnalysePic(input_img);

      // Result
      isaac_msgs::ImageInspectionResult result;

      switch (classification) {
        case vent_analysis_.vent_free_:
          result.result = "Vent free";
          result.response = RESPONSE::VENT_FREE;
          break;
        case vent_analysis_.vent_blocked_:
          result.result = "Vent has obstacle";
          result.response = RESPONSE::VENT_OBSTRUCTED;
          break;
        case vent_analysis_.vent_unknown_:
          result.result = "Unknown result, is robot looking at vent?";
          result.response = RESPONSE::INCONCLUSIVE;
          break;
        default:
          result.result = "Image analysis failed";
          result.response = RESPONSE::FAILED;
      }
      goal_.type = isaac_msgs::ImageInspectionGoal::NONE;
      // Send response
      server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
      break;
    }
    }
    return fsm_.Update(IMAGE_ANALYSED);
  }


  // IMAGE ANALYSIS ACTION SERVER

  // A new arm action has been called
  void GoalCallback(isaac_msgs::ImageInspectionGoalConstPtr const& goal) {
    ROS_ERROR_STREAM("RECEIVED()");
    goal_ = *goal;
    sci_cam_req_ = true;
    return fsm_.Update(IMAGE_WAIT);
  }

  // When the FSM state changes we get a callback here, so that we
  // can choose to do various things.
  void UpdateCallback(FSM::State const& state, FSM::Event const& event) {
    // Debug events
    isaac_msgs::ImageInspectionFeedback feedback;
    // Debug state changes
    switch (state) {
    case STATE::WAITING:
      feedback.state = STATE::WAITING;                      break;
    case STATE::ANALYSING:
      feedback.state = STATE::ANALYSING;                    break;
    }
    ROS_ERROR_STREAM("State changed to " << feedback.state);
    // Send the feedback if needed
    switch (state) {
    case STATE::WAITING:
      break;
    default:
      {
        ROS_ERROR_STREAM("send feedback");
        server_.SendFeedback(feedback);
      }
    }
  }

  // Complete the current inspection action
  void Result(int32_t response, std::string const& msg = "") {
    // Package up the feedback
    isaac_msgs::ImageInspectionResult result;
    result.result = msg;
    result.response = response;
    if (response > 0)
      server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    else if (response < 0)
      server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
    else
      server_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
  }

  // Callback to handle reconfiguration requests
  bool ReconfigureCallback(dynamic_reconfigure::Config & config) {
    // if ( fsm_.GetState() == STATE::WAITING)
    //   return cfg_.Reconfigure(config);
    return false;
  }

  // Preempt the current action with a new action
  void PreemptCallback() {
    ROS_ERROR("PreemptCallback");
  }

  // A Cancellation request arrives
  void CancelCallback() {
    ROS_ERROR("CancelCallback");
  }

 private:
  ff_util::FSM fsm_;
  ros::Publisher cmd_pub_;
  ros::Subscriber sub_sci_cam_;
  bool sci_cam_req_ = false;
  ff_util::FreeFlyerActionServer<isaac_msgs::ImageInspectionAction> server_;
  isaac_msgs::ImageInspectionGoal goal_;
  ff_util::ConfigServer cfg_;
  img_analysis::ImgVent vent_analysis_;
};

PLUGINLIB_EXPORT_CLASS(img_analysis::ImageAnalysisNode, nodelet::Nodelet);

}  // namespace img_analysis

