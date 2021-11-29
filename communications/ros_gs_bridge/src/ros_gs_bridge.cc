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

namespace ros_gs_bridge {

RosGsBridge::RosGsBridge() {
  requesting_control_ = false;
  grabbing_control_ = false;
  send_command_ = false;

  controller_ = "";
  isaac_apk_name_ = "gov.nasa.arc.irg.astrobee.isaac_gs_ros_bridge";
  isaac_controller_name_ = "ISAACGroundSystem";

  cmd_counter_ = 0;
  pub_queue_size_ = 10;
  sub_queue_size_ = 10;
}

RosGsBridge::~RosGsBridge() {
}

bool RosGsBridge::Initialize(ros::NodeHandle *nh) {
  nh_ = nh;

  // initialize actions
  img_ac_.SetResultCallback(
                          std::bind(&RosGsBridge::ImageInspectionResultCallback,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));
  img_ac_.Create(nh, ACTION_ANOMALY_IMG_ANALYSIS);

  inspection_as_.SetGoalCallback(std::bind(&RosGsBridge::InspectionGoalCallback,
                                           this,
                                           std::placeholders::_1));
  inspection_as_.SetPreemptCallback(
                              std::bind(&RosGsBridge::InspectionPreemptCallback,
                                        this));
  inspection_as_.SetCancelCallback(
                              std::bind(&RosGsBridge::InspectionCancelCallback,
                                        this));
  inspection_as_.Create(nh, ACTION_BEHAVIORS_INSPECTION);

  // initial services
  grab_control_srv_ = nh_->advertiseService(SERVICE_ROS_GS_BRIDGE_GRAB_CONTROL,
                                            &RosGsBridge::GrabControlService,
                                            this);

  // initialize subs
  sub_access_control_state_ = nh_->subscribe(
                                      TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE,
                                      sub_queue_size_,
                                      &RosGsBridge::AccessControlStateCallback,
                                      this);

  sub_gs_data_ = nh_->subscribe(TOPIC_GUEST_SCIENCE_DATA,
                                sub_queue_size_,
                                &RosGsBridge::GuestScienceDataCallback,
                                this);

  // initialize pubs
  cmd_pub_ = nh_->advertise<ff_msgs::CommandStamped>(TOPIC_COMMAND,
                                                     pub_queue_size_,
                                                     true);

  wifi_signals_pub_ = nh_->advertise<isaac_hw_msgs::WifiSignals>(
                                                            TOPIC_HARDWARE_WIFI,
                                                            pub_queue_size_,
                                                            true);

  // initialize what we can in the custom gs command
  custom_gs_command_.cmd_name =
                      ff_msgs::CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE;
  custom_gs_command_.cmd_src = isaac_controller_name_;
  custom_gs_command_.cmd_origin = "ground";
  custom_gs_command_.subsys_name = "GuestScience";

  // The custom guest science command takes two arguments, the apk name and
  // the command
  custom_gs_command_.args.resize(2);

  // Set the apk name and that the command is a string, the command string will
  // be set later
  custom_gs_command_.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  custom_gs_command_.args[0].s = isaac_apk_name_;
  custom_gs_command_.args[1].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;

  return true;
}

// Message Callbacks
void RosGsBridge::AccessControlStateCallback(
                    ff_msgs::AccessControlStateStampedConstPtr const& state) {
  controller_ = state->controller;

  // We only grab control when we need to send a guest science custom command
  if (requesting_control_) {
    requesting_control_ = false;
    GrabControl(state->cookie);
  } else if (grabbing_control_) {
    grabbing_control_ = false;

    if (HasControl()) {
      SendNextGSCommand();
    } else {
      // TODO(Katie): use ack instead
      // Grabbing control unsuccess full,
      ROS_ERROR("Unable to grab control.");
    }
  }
}

void RosGsBridge::GuestScienceDataCallback(
                                ff_msgs::GuestScienceDataConstPtr const& data) {
  // Make sure the data is from the isaac apk
  if (data->apk_name != isaac_apk_name_) {
    return;
  }

  // Make sure the data is data and not a json information message
  if (data->data_type != ff_msgs::GuestScienceData::STRING) {
    return;
  }

  std::string data_str(data->data.begin(), data->data.end());
  std::size_t delim_index = data_str.find("@");

  // Figure out what type of data is in the message
  if (data->topic == "iig") {
    // Data contains the image inspection goal
    isaac_msgs::ImageInspectionGoal goal;
    if (delim_index != std::string::npos) {
      goal.type = std::stoi(data_str.substr(0, delim_index));
      goal.img_topic = data_str.substr((delim_index + 1),
                                       (data_str.size() - delim_index - 1));
    } else {
      ROS_ERROR_STREAM("ros gs bridge: Error parsing image inspection goal." <<
                       " Data string is " << data_str);
      return;
    }

    // Send goal
    img_ac_.SendGoal(goal);
  } else if (data->topic == "if") {
    // Data contains the inspection feedback
    // TODO(Katie)
  } else if (data->topic == "ir") {
    // Data contains the inspection result
    isaac_msgs::InspectionResult result;
    int vent_size, geometry_size, response;

    // Get vent array
    if (delim_index == std::string::npos) {
      ROS_ERROR_STREAM("ros gs bridge: Error parsing vent size out of " <<
                       "inspection result. Data string is " << data_str);
      return;
    }

    vent_size = std::stoi(data_str.substr(0, delim_index));
    data_str.erase(0, (delim_index + 1));
    for (int i = 0; i < vent_size; i++) {
      delim_index = data_str.find("@");
      if (delim_index == std::string::npos) {
        ROS_ERROR_STREAM("ros gs bridge: Error parsing vent array out of " <<
                         "inspection result. Data string is " << data_str);
        return;
      }
      result.vent_result.push_back(std::stoi(data_str.substr(0, delim_index)));
      data_str.erase(0, (delim_index + 1));
    }

    // Get geometry array
    delim_index = data_str.find("@");
    if (delim_index == std::string::npos) {
      ROS_ERROR_STREAM("ros gs bridge: Error parsing geometry size out of " <<
                       "inspection result. Data string is " << data_str);
      return;
    }

    geometry_size = std::stoi(data_str.substr(0, delim_index));
    data_str.erase(0, (delim_index + 1));
    for (int i = 0; i < geometry_size; i++) {
      delim_index = data_str.find("@");
      if (delim_index == std::string::npos) {
        ROS_ERROR_STREAM("ros gs bridge: Error parsing geometry array out of" <<
                         " inspection result. Data string is " << data_str);
        return;
      }
      result.geometry_result.push_back(std::stoi(data_str.substr(0,
                                                                 delim_index)));
      data_str.erase(0, (delim_index + 1));
    }

    // Get response
    delim_index = data_str.find("@");
    if (delim_index == std::string::npos) {
      ROS_ERROR_STREAM("ros gs bridge: Error parsing response out of " <<
                       "inspection result. Data string is " << data_str);
      return;
    }

    response = std::stoi(data_str.substr(0, delim_index));
    data_str.erase(0, (delim_index + 1));
    result.response = response;

    // Get human readable response (fsm_result)
    result.fsm_result = data_str;

    if (response > 0) {
      inspection_as_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    } else if (response < 0) {
      inspection_as_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
    } else {
      inspection_as_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED,
                                result);
    }
  } else if (data->topic == "ws") {
    // Data contains the wifi signals msg
    isaac_hw_msgs::WifiSignals wifi_msg;
    int num_signals = 0;

    if (delim_index == std::string::npos) {
      ROS_ERROR_STREAM("ros gs bridge: Error parsing number of wifi signals." <<
                       "Data string is " << data_str);
      return;
    }

    num_signals = std::stoi(data_str.substr(0, delim_index));
    data_str.erase(0, (delim_index + 1));

    wifi_msg.signals.resize(num_signals);

    for (int i = 0; i < num_signals; i++) {
      // Get secs
      delim_index = data_str.find("@");
      if (delim_index == std::string::npos) {
        ROS_ERROR_STREAM("ros gs bridge: Error parsing timestamp seconds " <<
                         "out of wifi signals message. Data string is " <<
                         data_str);
        return;
      }

      wifi_msg.signals[i].header.stamp.sec =
                                    std::stoi(data_str.substr(0, delim_index));
      data_str.erase(0, (delim_index + 1));

      // Get nsecs
      delim_index = data_str.find("@");
      if (delim_index == std::string::npos) {
        ROS_ERROR_STREAM("ros gs bridge: Error parsing timestamp nseconds " <<
                         "out of wifi signals message. Data string is " <<
                         data_str);
        return;
      }

      wifi_msg.signals[i].header.stamp.nsec =
                                    std::stoi(data_str.substr(0, delim_index));
      data_str.erase(0, (delim_index + 1));

      // Get frame id
      delim_index = data_str.find("@");
      if (delim_index == std::string::npos) {
        ROS_ERROR_STREAM("ros gs bridge: Error parsing frame id out of wifi " <<
                         "signals message. Data string is " << data_str);
        return;
      }

      wifi_msg.signals[i].header.frame_id = data_str.substr(0, delim_index);
      data_str.erase(0, (delim_index + 1));

      // Get bssid
      delim_index = data_str.find("@");
      if (delim_index == std::string::npos) {
        ROS_ERROR_STREAM("ros gs bridge: Error parsing bssid out of wifi " <<
                         "signals message. Data string is " << data_str);
        return;
      }

      wifi_msg.signals[i].bssid = data_str.substr(0, delim_index);
      data_str.erase(0, (delim_index + 1));

      // Get ssid
      delim_index = data_str.find("@");
      if (delim_index == std::string::npos) {
        ROS_ERROR_STREAM("ros gs bridge: Error parsing ssid out of wifi " <<
                         "signals message. Data string is " << data_str);
        return;
      }

      wifi_msg.signals[i].ssid = data_str.substr(0, delim_index);
      data_str.erase(0, (delim_index + 1));

      // Get dBm
      delim_index = data_str.find("@");
      if (delim_index == std::string::npos) {
        // This means we are on the last element in the message
        wifi_msg.signals[i].signal_dbm = std::stoi(data_str);
      } else {
        wifi_msg.signals[i].signal_dbm =
                                    std::stoi(data_str.substr(0, delim_index));
        data_str.erase(0, (delim_index + 1));
      }
    }

    wifi_signals_pub_.publish(wifi_msg);
  } else {
    ROS_ERROR_STREAM("ros gs bridge: Unknown data topic " << data->topic <<
                     " in guest science data callback.");
  }
}

// Service Callbacks
bool RosGsBridge::GrabControlService(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res) {
  RequestControl();
  return true;
}

// Action Callbacks
void RosGsBridge::ImageInspectionResultCallback(
                      ff_util::FreeFlyerActionState::Enum const& state,
                      isaac_msgs::ImageInspectionResultConstPtr const& result) {
  // Time stamp in the result header is not used so we don't need to package it
  // TODO(Katie) to be thorough, it would be good to package up the timestamp
  // TODO(Katie) change string to be json
  std::string result_string = "iir@" + std::to_string(result->response) + "@";
  result_string += result->result + "\0";

  // TODO(Katie) expand to support multiple custom guest science commands
  if (result_string.size() >= 128) {
    ROS_ERROR_STREAM("ros gs brdige: Image inspection result string is too " <<
                     "big for a custom guest science command. Size is " <<
                     result_string.size());
  }

  // Set command argument
  custom_gs_command_.args[1].s = result_string;

  FillTimeIdInGSCmd();

  if (HasControl()) {
    cmd_pub_.publish(custom_gs_command_);
  } else {
    send_command_ = true;
    RequestControl();
  }
}

void RosGsBridge::InspectionGoalCallback(
                              isaac_msgs::InspectionGoalConstPtr const& goal) {
  // Time stamp in goal header and inspect poses don't seem to be used so we
  // don't need to package it
  // TODO(Katie) to be thorough, it would be good to package up the timestamps
  // TODO(Katie) change string to be json
  std::string goal_string = "ig@" + std::to_string(goal->command) + "@";

  for (int i = 0; i < goal->inspect_poses.poses.size(); i++) {
    goal_string += std::to_string(goal->inspect_poses.poses[i].position.x) + "@";
    goal_string += std::to_string(goal->inspect_poses.poses[i].position.y) + "@";
    goal_string += std::to_string(goal->inspect_poses.poses[i].position.z) + "@";
    goal_string += std::to_string(goal->inspect_poses.poses[i].orientation.x) + "@";
    goal_string += std::to_string(goal->inspect_poses.poses[i].orientation.y) + "@";
    goal_string += std::to_string(goal->inspect_poses.poses[i].orientation.z) + "@";
    goal_string += std::to_string(goal->inspect_poses.poses[i].orientation.w) + "@";
  }

  // Set last @ to the null character
  goal_string[(goal_string.size() - 1)] += '\0';

  // TODO(Katie) expand to support multiple custom guest science commands
  if (goal_string.size() >= 128) {
    ROS_ERROR_STREAM("ros gs bridge Inspection goal string is too big for a " <<
                     "custom guest science command. Size is " <<
                      goal_string.size());
    return;
  }

  // Set command argument
  custom_gs_command_.args[1].s = goal_string;

  FillTimeIdInGSCmd();

  if (HasControl()) {
    cmd_pub_.publish(custom_gs_command_);
  } else {
    send_command_ = true;
    RequestControl();
  }
}

void RosGsBridge::InspectionPreemptCallback() {
  ROS_ERROR("ros gs bridge: inspection preempt not implemented yet!");
}

void RosGsBridge::InspectionCancelCallback() {
  ROS_ERROR("ros gs bridge: inspection canceled not implemented yet!");
}

void RosGsBridge::FillTimeIdInGSCmd() {
  custom_gs_command_.header.stamp = ros::Time::now();
  custom_gs_command_.cmd_id = std::to_string(cmd_counter_) +
                              isaac_controller_name_ +
                              std::to_string(ros::Time::now().sec);

  cmd_counter_++;
}

void RosGsBridge::GrabControl(std::string cookie) {
  ff_msgs::CommandStamped gc_cmd;

  gc_cmd.header.stamp = ros::Time::now();
  gc_cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_GRAB_CONTROL;
  gc_cmd.cmd_id = std::to_string(cmd_counter_) + isaac_controller_name_ +
                  std::to_string(ros::Time::now().sec);
  gc_cmd.cmd_src = isaac_controller_name_;
  gc_cmd.cmd_origin = "ground";
  gc_cmd.subsys_name = "AccessControl";

  // The grab control command has one argument, the cookie
  gc_cmd.args.resize(1);
  gc_cmd.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  gc_cmd.args[0].s = cookie;

  cmd_counter_++;

  cmd_pub_.publish(gc_cmd);
  grabbing_control_ = true;
}

bool RosGsBridge::HasControl() {
  if (controller_ == isaac_controller_name_) {
    return true;
  }

  return false;
}

void RosGsBridge::RequestControl() {
  ff_msgs::CommandStamped rc_cmd;

  rc_cmd.header.stamp = ros::Time::now();
  rc_cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_REQUEST_CONTROL;
  rc_cmd.cmd_id = std::to_string(cmd_counter_) + isaac_controller_name_ +
                  std::to_string(ros::Time::now().sec);
  rc_cmd.cmd_src = isaac_controller_name_;
  rc_cmd.cmd_origin = "ground";
  rc_cmd.subsys_name = "AccessControl";

  cmd_counter_++;

  cmd_pub_.publish(rc_cmd);
  requesting_control_ = true;
}

void RosGsBridge::SendNextGSCommand() {
  // TODO(Katie) Right now this only supports one guest science custom command
  // for the data in an action. When this is changed, this function will need
  // to be completely rewritten
  if (send_command_) {
    cmd_pub_.publish(custom_gs_command_);
    send_command_ = false;
  }
}
}  // namespace ros_gs_bridge
