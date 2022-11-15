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
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>
#include <ff_util/config_server.h>
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_fsm.h>
#include <isaac_util/isaac_names.h>

// Hardware messages
#include <sensor_msgs/CompressedImage.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <isaac_msgs/SciCamAction.h>

/**
 * \ingroup hw
 */
namespace sci_cam {

using RESPONSE = isaac_msgs::SciCamResult;

class SciCamNode : public ff_util::FreeFlyerNodelet {
 public:
  SciCamNode() : ff_util::FreeFlyerNodelet(NODE_SCI_CAM) {}
  ~SciCamNode() {}

 protected:
  // Called on flight software stack initialization - every NODELET_FATAIL
  // call below should be converted to an initialization fault...
  void Initialize(ros::NodeHandle *nh) {
    nh_ = nh;
    // Set the config path to ISAAC
    char *path = getenv("CUSTOM_CONFIG_DIR");
    if (path != NULL)
      cfg_.SetPath(path);
    // Grab some configuration parameters for this node from the LUA config reader
    cfg_.Initialize(GetPrivateHandle(), "behaviors/sci_cam.config");
    if (!cfg_.Listen(boost::bind(
      &SciCamNode::ReconfigureCallback, this, _1)))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                         "Could not load config");

    // Publish the guest science command fr the sci cam
    pub_guest_sci_ = nh->advertise<ff_msgs::CommandStamped>(
      TOPIC_COMMAND, 1, true);

    // Subscribe to the sci camera topic to make sure a picture was taken
    sub_sci_cam_ = nh->subscribe(TOPIC_HARDWARE_SCI_CAM + std::string("/compressed"), 1,
                      &SciCamNode::SciCamCallback, this);

    // Timer for the sci cam camera
    sci_cam_timeout_ = nh->createTimer(ros::Duration(cfg_.Get<double>("sci_cam_timeout")), &SciCamNode::SciCamTimeout,
                                        this, false, false);

    // Setup the sci_cam action
    server_.SetGoalCallback(std::bind(
      &SciCamNode::GoalCallback, this, std::placeholders::_1));
    server_.SetPreemptCallback(std::bind(
      &SciCamNode::PreemptCallback, this));
    server_.SetCancelCallback(std::bind(
      &SciCamNode::CancelCallback, this));
    server_.Create(nh, ACTION_BEHAVIORS_SCI_CAM);
  }

  // Send an apk command
  bool TakePic() {
    // Allow image to stabilize
    ros::Duration(cfg_.Get<double>("station_time")).sleep();

    // Signal an imminent sci cam image
    sci_cam_req_ = sci_cam_req_ + 1;

    // Take picture
    ff_msgs::CommandArg arg;
    std::vector<ff_msgs::CommandArg> cmd_args;

    // The command sends two strings. The first has the app name,
    // and the second the command value, encoded as json.

    arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
    arg.s = "gov.nasa.arc.irg.astrobee.sci_cam_image";
    cmd_args.push_back(arg);

    arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
    arg.s = "{\"name\": \"takePicture\"}";
    cmd_args.push_back(arg);

    ff_msgs::CommandStamped cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE;
    cmd.cmd_id = "inspection" + std::to_string(ros::Time::now().toSec());
    cmd.cmd_src = "guest science";
    cmd.cmd_origin = "guest science";
    cmd.args = cmd_args;


    pub_guest_sci_.publish(cmd);


    // Timer for the sci cam camera
    sci_cam_timeout_.start();

    return 0;
  }

  void SciCamCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    // The sci cam image was received
    if (sci_cam_req_ != 0) {
      // Clear local variables
      sci_cam_timeout_.stop();
      sci_cam_req_ = 0;
      // result_.inspection_result.push_back(isaac_msgs::InspectionResult::PIC_ACQUIRED);
      // result_.picture_time.push_back(msg->header.stamp);

      ROS_DEBUG_STREAM("Scicam picture acquired " << ros::Time::now());
      // return fsm_.Update(NEXT_INSPECT);
    }
    return;
  }
  void SciCamTimeout(const ros::TimerEvent& event) {
    sci_cam_timeout_.stop();
    // The sci cam image was not received
    if (sci_cam_req_ < 2) {
      ROS_WARN_STREAM("sci cam didn't repond, resending it again");
      TakePic();
      return;
    } else {
      // return fsm_.Update(INSPECT_FAILED);
    }
  }

    // INSPECTION ACTION SERVER

  // A new arm action has been called
  void GoalCallback(isaac_msgs::SciCamGoalConstPtr const& goal) {
    // Save new goal
    goal_ = *goal;

    // Identify command
    switch (goal_.command) {
    // Single picture command
    case isaac_msgs::SciCamGoal::SINGLE_PIC:
      NODELET_DEBUG("Received Goal Vent");
      TakePic();
      break;
    // Continuous picture taking command
    case isaac_msgs::SciCamGoal::CONT_PIC:
      NODELET_DEBUG("Received Goal Geometry");
      break;

    // Invalid command
    default:
    {
      isaac_msgs::SciCamResult result;
      result.fsm_result = "Invalid command in request";
      result.response = RESPONSE::INVALID_COMMAND;
      server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
      break;
    }
    }
  }

  // Complete the current inspection action
  void Result(int32_t response, std::string const& msg = "") {
    // Package up the feedback
    result_.fsm_result = msg;
    result_.response = response;
    if (response > 0)
      server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result_);
    else if (response < 0)
      server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result_);
    else
      server_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result_);
  }

  // Callback to handle reconfiguration requests
  bool ReconfigureCallback(dynamic_reconfigure::Config & config) {
    return cfg_.Reconfigure(config);
  }

  // When in continuos picture taking with each received image we get a callback here.
  void UpdateCallback() {
    // Debug events
    ROS_DEBUG_STREAM("InspectionFeedback ");
    isaac_msgs::SciCamFeedback feedback;
    feedback.feedback = 1;
    server_.SendFeedback(feedback);
  }

  // Preempt the current action with a new action
  void PreemptCallback() {
    NODELET_DEBUG_STREAM("PreemptCallback");
    return;
  }

  // A Cancellation request arrives
  void CancelCallback() {
    NODELET_DEBUG_STREAM("CancelCallback");
    return;
  }

 private:
  ros::NodeHandle* nh_;
  ff_util::FreeFlyerActionServer<isaac_msgs::SciCamAction> server_;
  ff_util::ConfigServer cfg_;
  ros::Publisher pub_guest_sci_;
  ros::Subscriber sub_sci_cam_;
  ros::Timer sci_cam_timeout_;
  isaac_msgs::SciCamGoal goal_;
  isaac_msgs::SciCamResult result_;
  // Flag to wait for sci camera
  int sci_cam_req_ = 0;
};

PLUGINLIB_EXPORT_CLASS(sci_cam::SciCamNode, nodelet::Nodelet);

}  // namespace sci_cam
