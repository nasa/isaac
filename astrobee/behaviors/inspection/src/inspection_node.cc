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
#include <ros/package.h>

// Include inspection library header
#include <inspection/inspection.h>

// TF2 support
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Shared project includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_service.h>
#include <ff_util/ff_fsm.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>
#include <isaac_util/isaac_names.h>

// Software messages
#include <sensor_msgs/CompressedImage.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <isaac_msgs/InspectionState.h>

// Services
#include <ff_msgs/SetState.h>

// Actions
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/DockAction.h>
#include <isaac_msgs/InspectionAction.h>
#include <isaac_msgs/ImageInspectionAction.h>

// Eigen for math
#include <Eigen/Dense>

typedef actionlib::SimpleActionServer<isaac_msgs::InspectionAction> Server;

/**
 * \ingroup beh
 */
namespace inspection {

// Match the internal states and responses with the message definition
using FSM = ff_util::FSM;
using STATE = isaac_msgs::InspectionState;
using RESPONSE = isaac_msgs::InspectionResult;

/*
  This class provides the high-level logic that allows the freeflyer to
  perform inspection, detecting any anomaly. Please see the accompanying
  state diagram for a high level overview of this logic.

  Here are the rules of inspection:

    * Vent blocked: the freeflyer will approach the possibly obstructed
    vent and run image analysis to detect if the vent is obstructed.

    * Other: todo
*/
class InspectionNode : public ff_util::FreeFlyerNodelet {
 public:
  // All possible events that can occur
  enum : FSM::Event {
    READY            = (1<<0),    // System is initialized
    GOAL_INSPECT     = (1<<1),    // Goal to start inspection
    GOAL_CANCEL      = (1<<2),    // Cancel an existing goal
    GOAL_PREEMPT     = (1<<3),    // Preempt an existing goal
    GOAL_PAUSE       = (1<<4),    // Pause an existing goal
    GOAL_UNPAUSE     = (1<<5),    // Resume an existing goal
    MOTION_SUCCESS   = (1<<6),    // Mobility motion action success
    MOTION_FAILED    = (1<<7),    // Mobility motion action problem
    MOTION_UPDATE    = (1<<8),    // Mobility motion update
    DOCK_SUCCESS     = (1<<9),    // Dock motion action success
    DOCK_FAILED      = (1<<10),   // Dock motion action problem
    DOCK_UPDATE      = (1<<11),   // Dock motion update
    NEXT_INSPECT     = (1<<12),   // Visual Inspection action success
    INSPECT_FAILED   = (1<<13),   // Visual inspection action problem
    INSPECT_UPDATE   = (1<<14),   // Visual inspection update
    MANUAL_STATE_SET = (1<<15)    // Setting the state manually with service
  };

  InspectionNode() : ff_util::FreeFlyerNodelet(NODE_INSPECTION, false),
    fsm_(STATE::INITIALIZING, std::bind(&InspectionNode::UpdateCallback,
      this, std::placeholders::_1, std::placeholders::_2)) {
    // Add the state transition lambda functions - refer to the FSM diagram
    // [0]
    fsm_.Add(STATE::INITIALIZING,
      READY, [this](FSM::Event const& event) -> FSM::State {
        return STATE::WAITING;
      });
    // [1]
    fsm_.Add(STATE::WAITING,
      GOAL_INSPECT | GOAL_UNPAUSE, [this](FSM::Event const& event) -> FSM::State {
        Dock("UNDOCK");
        return STATE::INIT_INSPECTION;
      });
    // [2]
    fsm_.Add(STATE::INIT_INSPECTION,
      DOCK_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        switch (goal_.command) {
        // Vent command
        case isaac_msgs::InspectionGoal::ANOMALY:
          if (inspection_->GenSegment(goal_.inspect_poses.poses[goal_counter_])) {
            MoveInspect(ff_msgs::MotionGoal::NOMINAL, inspection_->GetInspectionPose());
            return STATE::MOVING_TO_APPROACH_POSE;
          } else {
            Result(RESPONSE::MOTION_APPROACH_FAILED);
            return STATE::WAITING;
          }
        // Geometry or Panorama command
        case isaac_msgs::InspectionGoal::GEOMETRY:
        case isaac_msgs::InspectionGoal::PANORAMA:
          MoveInspect(ff_msgs::MotionGoal::NOMINAL, goal_.inspect_poses.poses[goal_counter_]);
          return STATE::MOVING_TO_APPROACH_POSE;
        // Volumetric command
        case isaac_msgs::InspectionGoal::VOLUMETRIC:
          MoveInspect(ff_msgs::MotionGoal::NOMINAL, goal_.inspect_poses.poses[goal_counter_]);
          return STATE::VOL_INSPECTION;
        }
        return STATE::WAITING;
      });
    fsm_.Add(STATE::INIT_INSPECTION,
      DOCK_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::DOCK_FAILED);
        return STATE::WAITING;
      });
    // [3]
    fsm_.Add(STATE::MOVING_TO_APPROACH_POSE,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        ImageInspect();
        return STATE::VISUAL_INSPECTION;
      });
    // [4]
    fsm_.Add(STATE::VISUAL_INSPECTION,
      NEXT_INSPECT, [this](FSM::Event const& event) -> FSM::State {
        // After successful inspection, can increment the counter
        goal_counter_++;
        // There is more to inspect
        if (goal_counter_ < goal_.inspect_poses.poses.size()) {
          switch (goal_.command) {
          // Vent command
          case isaac_msgs::InspectionGoal::ANOMALY:
            if (inspection_->GenSegment(goal_.inspect_poses.poses[goal_counter_])) {
              MoveInspect(ff_msgs::MotionGoal::NOMINAL, inspection_->GetInspectionPose());
              return STATE::MOVING_TO_APPROACH_POSE;
            } else {
              Result(RESPONSE::MOTION_APPROACH_FAILED);
              return STATE::WAITING;
            }
          case isaac_msgs::InspectionGoal::GEOMETRY:
          case isaac_msgs::InspectionGoal::PANORAMA:
            MoveInspect(ff_msgs::MotionGoal::NOMINAL, goal_.inspect_poses.poses[goal_counter_]);
            return STATE::MOVING_TO_APPROACH_POSE;
          }
        }
        // Inspection is over, return
        Result(RESPONSE::SUCCESS, "Inspection Over");
        return STATE::WAITING;
      });
    // [5]
    fsm_.Add(STATE::VOL_INSPECTION,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        // After successful inspection, can increment the counter
        goal_counter_++;
        // There is more to inspect
        if (goal_counter_ < goal_.inspect_poses.poses.size()) {
          MoveInspect(ff_msgs::MotionGoal::NOMINAL, goal_.inspect_poses.poses[goal_counter_]);
          return STATE::VOL_INSPECTION;
        }
        // Inspection is over, return
        Result(RESPONSE::SUCCESS, "Inspection Over");
        Dock("DOCK");
        return STATE::RETURN_INSPECTION;
      });
    // [6]
    fsm_.Add(STATE::RETURN_INSPECTION,
      DOCK_SUCCESS | DOCK_FAILED | MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        return STATE::WAITING;
      });
    // [7]
    fsm_.Add(MOTION_FAILED | INSPECT_FAILED,
      [this](FSM::State const& state, FSM::Event const& event) -> FSM::State {
        switch (event) {
        case MOTION_FAILED:
          Result(RESPONSE::MOTION_APPROACH_FAILED);
          break;
        case INSPECT_FAILED:
          Result(RESPONSE::VISUAL_INSPECTION_FAILED);
          break;
        }
        return STATE::WAITING;
      });
    //////////////////////////////////////////////////////
    // CATCH-ALL FOR CANCELLATIONS / PREEMPTION / PAUSE //
    //////////////////////////////////////////////////////
    fsm_.Add(GOAL_CANCEL | GOAL_PREEMPT | GOAL_PAUSE,
      [this](FSM::State const& state, FSM::Event const& event) -> FSM::State {
        switch (state) {
        // Dock/Undock in progress
        case STATE::INIT_INSPECTION:
          client_d_.CancelGoal();
          break;
        // Motion in progress
        case STATE::MOVING_TO_APPROACH_POSE:
          client_m_.CancelGoal();
          break;
        case STATE::RETURN_INSPECTION:
          client_m_.CancelGoal();
          client_d_.CancelGoal();
          break;
        // Visual Inspection in progress
        case STATE::VISUAL_INSPECTION:
          break;
        }
        // After cancellations, wait
        return STATE::WAITING;
      });
  }

  ~InspectionNode() {}

 protected:
  void Initialize(ros::NodeHandle* nh) {
    nh_ = nh;
    // Set the config path to ISAAC
    char *path = getenv("CUSTOM_CONFIG_DIR");
    if (path != NULL)
      cfg_.SetPath(path);
    // Grab some configuration parameters for this node from the LUA config reader
    cfg_.Initialize(GetPrivateHandle(), "behaviors/inspection.config");
    if (!cfg_.Listen(boost::bind(
      &InspectionNode::ReconfigureCallback, this, _1)))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                         "Could not load config");

    // Publish the inspection state as a latched topic
    pub_state_ = nh->advertise<isaac_msgs::InspectionState>(
      TOPIC_BEHAVIORS_INSPECTION_STATE, 1, true);

    // Publish the guest science command fr the sci cam
    pub_guest_sci_ = nh->advertise<ff_msgs::CommandStamped>(
      TOPIC_COMMAND, 1, true);

    // Subscribe to the sci camera topic to make sure a picture was taken
    sub_sci_cam_ = nh->subscribe(TOPIC_HARDWARE_SCI_CAM + std::string("/compressed"), 1,
                      &InspectionNode::SciCamCallback, this);

    // Allow the state to be manually set
    server_set_state_ = nh->advertiseService(SERVICE_BEHAVIORS_INSPECTION_SET_STATE,
      &InspectionNode::SetStateCallback, this);

    // Setup move client action
    client_m_.SetConnectedTimeout(cfg_.Get<double>("timeout_motion_connected"));
    client_m_.SetActiveTimeout(cfg_.Get<double>("timeout_motion_active"));
    client_m_.SetResponseTimeout(cfg_.Get<double>("timeout_motion_response"));
    client_m_.SetFeedbackCallback(std::bind(&InspectionNode::MFeedbackCallback,
      this, std::placeholders::_1));
    client_m_.SetResultCallback(std::bind(&InspectionNode::MResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_m_.SetConnectedCallback(std::bind(
      &InspectionNode::ConnectedCallback, this));
    client_m_.Create(nh, ACTION_MOBILITY_MOTION);

    // Setup dock client action
    client_d_.SetConnectedTimeout(cfg_.Get<double>("timeout_dock_connected"));
    client_d_.SetActiveTimeout(cfg_.Get<double>("timeout_dock_active"));
    client_d_.SetResponseTimeout(cfg_.Get<double>("timeout_dock_response"));
    client_d_.SetFeedbackCallback(std::bind(&InspectionNode::DFeedbackCallback,
      this, std::placeholders::_1));
    client_d_.SetResultCallback(std::bind(&InspectionNode::DResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_d_.SetConnectedCallback(std::bind(
      &InspectionNode::ConnectedCallback, this));
    client_d_.Create(nh, ACTION_BEHAVIORS_DOCK);

    // Get parameter whether we running this in simulation or the robot
    ros::param::get("sim_mode", sim_mode_);

    // Setup image analysis client action
    client_i_.SetConnectedTimeout(cfg_.Get<double>("timeout_image_analysis_connected"));
    client_i_.SetActiveTimeout(cfg_.Get<double>("timeout_image_analysis_active"));
    client_i_.SetResponseTimeout(cfg_.Get<double>("timeout_image_analysis_response"));
    client_i_.SetFeedbackCallback(std::bind(&InspectionNode::IFeedbackCallback,
      this, std::placeholders::_1));
    client_i_.SetResultCallback(std::bind(&InspectionNode::IResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_i_.SetConnectedCallback(std::bind(
      &InspectionNode::GroundConnectedCallback, this));
    client_i_.Create(nh, ACTION_ANOMALY_IMG_ANALYSIS);

    // Setup the execute action
    server_.SetGoalCallback(std::bind(
      &InspectionNode::GoalCallback, this, std::placeholders::_1));
    server_.SetPreemptCallback(std::bind(
      &InspectionNode::PreemptCallback, this));
    server_.SetCancelCallback(std::bind(
      &InspectionNode::CancelCallback, this));
    server_.Create(nh, ACTION_BEHAVIORS_INSPECTION);

    // Read maximum number of retryals for a motion action
    max_motion_retry_number_= cfg_.Get<int>("max_motion_retry_number");

    // Initiate inspection library
    inspection_ = new Inspection(nh, &cfg_);
  }

  // Ensure all clients are connected
  void ConnectedCallback() {
    NODELET_DEBUG_STREAM("ConnectedCallback()");
    if (!client_m_.IsConnected()) return;       // Move action
    if (!client_d_.IsConnected()) return;       // Dock action
    fsm_.Update(READY);                         // Ready!
  }

  void GroundConnectedCallback() {
    ROS_DEBUG_STREAM("GroundConnectedCallback()");
    if (!client_i_.IsConnected()) return;       // Move action
    ground_active_ = true;
  }

  // Service callback to manually change the state
  bool SetStateCallback(ff_msgs::SetState::Request& req,
                        ff_msgs::SetState::Response& res) {
    fsm_.SetState(req.state);
    res.success = true;
    UpdateCallback(fsm_.GetState(), MANUAL_STATE_SET);
    return true;
  }


  // Send a move command
  bool MoveInspect(std::string const& mode, geometry_msgs::Pose pose) {
    // Create a new motion goal
    ff_msgs::MotionGoal goal;
    goal.command = ff_msgs::MotionGoal::MOVE;
    goal.flight_mode = mode;

    // Package up the desired end pose
    geometry_msgs::PoseStamped inspection_pose;
    inspection_pose.pose = pose;
    inspection_pose.header.stamp = ros::Time::now();
    goal.states.push_back(inspection_pose);

    // Load parameters from config file
    std::string planner = cfg_.Get<std::string>("planner");
    bool coll_check = cfg_.Get<bool>("enable_collision_checking");

    bool replanning = cfg_.Get<bool>("enable_replanning");
    int replanning_attempts = cfg_.Get<int>("max_replanning_attempts");
    bool validation = cfg_.Get<bool>("enable_validation");
    bool boostrapping = cfg_.Get<bool>("enable_bootstrapping");
    bool immediate = cfg_.Get<bool>("enable_immediate");

    // Reconfigure the choreographer
    ff_util::ConfigClient choreographer_cfg(GetPlatformHandle(), NODE_CHOREOGRAPHER);
    choreographer_cfg.Set<std::string>("planner", planner);
    choreographer_cfg.Set<bool>("enable_collision_checking", coll_check);
    switch (goal_.command) {
    // Vent command
    case isaac_msgs::InspectionGoal::ANOMALY:
      choreographer_cfg.Set<bool>("enable_faceforward",
              cfg_.Get<bool>("enable_faceforward_anomaly"));    break;
    case isaac_msgs::InspectionGoal::GEOMETRY:
      choreographer_cfg.Set<bool>("enable_faceforward",
              cfg_.Get<bool>("enable_faceforward_geometry"));    break;
    }
    choreographer_cfg.Set<bool>("enable_replanning", replanning);
    choreographer_cfg.Set<int>("max_replanning_attempts", replanning_attempts);
    choreographer_cfg.Set<bool>("enable_validation", validation);
    choreographer_cfg.Set<bool>("enable_bootstrapping", boostrapping);
    choreographer_cfg.Set<bool>("enable_immediate", immediate);
    if (!choreographer_cfg.Reconfigure()) {
      NODELET_ERROR_STREAM("Failed to reconfigure choreographer");
      return false;
    }
    // Send the goal to the mobility subsystem
    return client_m_.SendGoal(goal);
  }

  // Ignore the move feedback, for now
  void MFeedbackCallback(ff_msgs::MotionFeedbackConstPtr const& feedback) {
    m_fsm_subevent_ = feedback->state.fsm_event;
    m_fsm_substate_ = feedback->state.fsm_state;
    UpdateCallback(fsm_.GetState(), MOTION_UPDATE);
  }

  // Result of a move action
  void MResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    ff_msgs::MotionResultConstPtr const& result) {
    switch (result_code) {
      case ff_util::FreeFlyerActionState::SUCCESS:
        motion_retry_number_ = 0;
        return fsm_.Update(MOTION_SUCCESS);
    }
    if (result != nullptr) {
      switch (result->response) {
        case ff_msgs::MotionResult::PLAN_FAILED:
        case ff_msgs::MotionResult::VALIDATE_FAILED:
        case ff_msgs::MotionResult::OBSTACLE_DETECTED:
        case ff_msgs::MotionResult::REPLAN_NOT_ENOUGH_TIME:
        case ff_msgs::MotionResult::REPLAN_FAILED:
        case ff_msgs::MotionResult::REVALIDATE_FAILED:
        case ff_msgs::MotionResult::VIOLATES_KEEP_OUT:
        case ff_msgs::MotionResult::VIOLATES_KEEP_IN:
        {
          // Try to find an alternate inspection position
          if (inspection_->RemoveInspectionPose()) {
            MoveInspect(ff_msgs::MotionGoal::NOMINAL, inspection_->GetInspectionPose());
            return;
          } else {
            ROS_ERROR_STREAM("No inspection pose possible for selected target");
          }
          break;
        }
        case ff_msgs::MotionResult::TOLERANCE_VIOLATION_POSITION:
        case ff_msgs::MotionResult::TOLERANCE_VIOLATION_ATTITUDE:
        case ff_msgs::MotionResult::TOLERANCE_VIOLATION_VELOCITY:
        case ff_msgs::MotionResult::TOLERANCE_VIOLATION_OMEGA:
        {  // If it fails because of a motion error, retry
          if (motion_retry_number_ < max_motion_retry_number_) {
            motion_retry_number_++;
            switch (goal_.command) {
            // Vent command
            case isaac_msgs::InspectionGoal::ANOMALY:
              MoveInspect(ff_msgs::MotionGoal::NOMINAL, inspection_->GetInspectionPose());
              break;
            case isaac_msgs::InspectionGoal::GEOMETRY:
            case isaac_msgs::InspectionGoal::VOLUMETRIC:
              MoveInspect(ff_msgs::MotionGoal::NOMINAL, goal_.inspect_poses.poses[goal_counter_]);
              break;
            }
          }
        }
      }

      ROS_ERROR_STREAM("Motion failed result error: " << result->response);
    } else {
      ROS_ERROR_STREAM("Invalid result received Motion");
    }
    return fsm_.Update(MOTION_FAILED);
  }

  // Dock ACTION CLIENT

  // Send a dock or undock command
  bool Dock(std::string const& mode) {
    // Create a new motion goal
    ff_msgs::DockGoal goal;
    if (mode == "DOCK") {
      goal.command = ff_msgs::DockGoal::DOCK;
      goal.berth = ff_msgs::DockGoal::BERTH_1;
    } else if (mode == "UNDOCK") {
      goal.command = ff_msgs::DockGoal::UNDOCK;
    }

    // Send the goal to the dock behavior
    return client_d_.SendGoal(goal);
  }

  // Ignore the move feedback, for now
  void DFeedbackCallback(ff_msgs::DockFeedbackConstPtr const& feedback) {
    d_fsm_subevent_ = feedback->state.fsm_event;
    d_fsm_substate_ = feedback->state.fsm_state;
    UpdateCallback(fsm_.GetState(), DOCK_UPDATE);

    dock_state_ = feedback->state;
  }

  // Result of a move action
  void DResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
  ff_msgs::DockResultConstPtr const& result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      return fsm_.Update(DOCK_SUCCESS);
    default:
      return fsm_.Update(DOCK_FAILED);
    }
  }

  // IMG ANALYSIS

  // Send a move command
  bool ImageInspect() {
    // Activate image anomaly detector if there are gound communications
    if (goal_.command == isaac_msgs::InspectionGoal::ANOMALY && ground_active_) {
      // Send goal
      isaac_msgs::ImageInspectionGoal goal;
      goal.type = isaac_msgs::ImageInspectionGoal::VENT;
      client_i_.SendGoal(goal);
      ROS_ERROR_STREAM("sent image inspection goal");
    }

    // Allow image to stabilize
    ros::Duration(cfg_.Get<double>("station_time")).sleep();

    // Signal an imminent sci cam image
    sci_cam_req_ = true;

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
    ros::Timer sci_cam_timeout_ = nh_->createTimer(ros::Duration(5), &InspectionNode::SciCamTimeout, this, true, false);
    return 0;
  }

  void SciCamCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    // The sci cam image was received
    if (sci_cam_req_) {
      sci_cam_req_ = false;
      result_.inspection_result.push_back(isaac_msgs::InspectionResult::PIC_ACQUIRED);
      result_.picture_time.push_back(msg->header.stamp);

      if (goal_.command == isaac_msgs::InspectionGoal::ANOMALY && ground_active_) {
        ROS_ERROR_STREAM("wait for result()");
        return;
      }
      ROS_DEBUG_STREAM("Scicam picture acquired " << ros::Time::now());
      return fsm_.Update(NEXT_INSPECT);
    }
    return;
  }
  void SciCamTimeout(const ros::TimerEvent& event) {
    // The sci cam image was not received
    return fsm_.Update(INSPECT_FAILED);
  }

  // Feedback of an action
  void IFeedbackCallback(isaac_msgs::ImageInspectionFeedbackConstPtr const& feedback) {
    ROS_DEBUG_STREAM("IFeedbackCallback()");
  }

  // Result of an action
  void IResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    isaac_msgs::ImageInspectionResultConstPtr const& result) {
    ROS_DEBUG_STREAM("IResultCallback()");
    // Fill in the result message with the result
    if (result != nullptr)
      result_.anomaly_result.push_back(result->anomaly_result);
    else
      ROS_ERROR_STREAM("Invalid result received Image Analysis");
    return fsm_.Update(NEXT_INSPECT);
  }

  // INSPECTION ACTION SERVER

  // A new arm action has been called
  void GoalCallback(isaac_msgs::InspectionGoalConstPtr const& goal) {
    if (!goal_.inspect_poses.poses.empty()) {
      isaac_msgs::InspectionResult result;
      switch (goal->command) {
      // Pause command
      case isaac_msgs::InspectionGoal::PAUSE:
          result.fsm_result = "Pausing";
          result.response = RESPONSE::SUCCESS;
          server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
        return fsm_.Update(GOAL_PAUSE);
      // Resume command
      case isaac_msgs::InspectionGoal::RESUME:
        return fsm_.Update(GOAL_INSPECT);
      // Skip command
      case isaac_msgs::InspectionGoal::SKIP:
      {
        if (!goal_.inspect_poses.poses.empty() && goal_counter_ < goal_.inspect_poses.poses.size() - 1) {
          // Skip the current pose
          goal_counter_++;
          result.fsm_result = "Skipped pose";
          result.response = RESPONSE::SUCCESS;
          server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
          return fsm_.Update(GOAL_PAUSE);
        } else {
          result.fsm_result = "Nothing to skip";
          result.response = RESPONSE::INVALID_COMMAND;
          server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
          return fsm_.Update(GOAL_PAUSE);
        }
        return;
      }
      // Repeat last executed step command
      case isaac_msgs::InspectionGoal::REPEAT:
      {
        if (!goal_.inspect_poses.poses.empty() && goal_counter_ > 0) {
          // Go back to the last pose
          goal_counter_--;

          result.fsm_result = "Will repeat last pose";
          result.response = RESPONSE::SUCCESS;
          server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
          return fsm_.Update(GOAL_PAUSE);
        } else {
          result.fsm_result = "Nothing to repeat";
          result.response = RESPONSE::INVALID_COMMAND;
          server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
          return fsm_.Update(GOAL_PAUSE);
        }
        return;
      }
      // Save command
      case isaac_msgs::InspectionGoal::SAVE:
        if (!goal_.inspect_poses.poses.empty() && goal_counter_ < goal_.inspect_poses.poses.size()) {
          std::ofstream myfile;
          std::string path = ros::package::getPath("inspection") + "/resources/current.txt";
          myfile.open(path);
          for (int i = goal_counter_; i < goal_.inspect_poses.poses.size(); i++) {
            myfile << goal_.inspect_poses.poses[i].position.x << " "
                   << goal_.inspect_poses.poses[i].position.y << " "
                   << goal_.inspect_poses.poses[i].position.z << " "
                   << goal_.inspect_poses.poses[i].orientation.x << " "
                   << goal_.inspect_poses.poses[i].orientation.y << " "
                   << goal_.inspect_poses.poses[i].orientation.z << " "
                   << goal_.inspect_poses.poses[i].orientation.w << "\n";
          }
          myfile.close();
          result.fsm_result = "Saved";
          result.response = RESPONSE::SUCCESS;
          server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
          return fsm_.Update(GOAL_PAUSE);
        } else {
          NODELET_ERROR_STREAM("Nothing to save");
          result.fsm_result = "Nothing to save";
          result.response = RESPONSE::FAILED;
          server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
          return fsm_.Update(GOAL_PAUSE);
        }
        return;
      }
    }
    // Save new goal
    goal_ = *goal;
    result_.anomaly_result.clear();
    result_.inspection_result.clear();
    result_.picture_time.clear();
    goal_counter_ = 0;
    ROS_DEBUG_STREAM("RESET COUNTER");

    // Check if there is at least one valid inspection pose
    if (goal_.inspect_poses.poses.empty()) {
      isaac_msgs::InspectionResult result;
      result.fsm_result = "No inspection pose added / nothing to pause or resume";
      result.response = RESPONSE::INVALID_COMMAND;
      server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
      return;
    }
    // Identify command
    switch (goal_.command) {
    // Vent command
    case isaac_msgs::InspectionGoal::ANOMALY:
      NODELET_DEBUG("Received Goal Vent");
      return fsm_.Update(GOAL_INSPECT);
      break;
    // Geometry command
    case isaac_msgs::InspectionGoal::GEOMETRY:
      NODELET_DEBUG("Received Goal Geometry");
      return fsm_.Update(GOAL_INSPECT);
      break;
    // Panorama command
    case isaac_msgs::InspectionGoal::PANORAMA:
      ROS_ERROR("Received Goal Panorama");
      inspection_->GeneratePanoramaSurvey(goal_.inspect_poses);
      return fsm_.Update(GOAL_INSPECT);
      break;
    // Volumetric command
    case isaac_msgs::InspectionGoal::VOLUMETRIC:
      NODELET_DEBUG("Received Goal Volumetric");
      return fsm_.Update(GOAL_INSPECT);
      break;
    // Invalid command
    default:
    {
      isaac_msgs::InspectionResult result;
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
    if (cfg_.Reconfigure(config)) {
      inspection_->ReadParam();
      return true;
    }
    return false;
  }

  // When the FSM state changes we get a callback here, so that we
  // can choose to do various things.
  void UpdateCallback(FSM::State const& state, FSM::Event const& event) {
    // Debug events
    isaac_msgs::InspectionState msg;
    msg.header.frame_id = GetPlatform();
    msg.header.stamp = ros::Time::now();
    msg.state = state;
    // Debug events
    switch (event) {
    case READY:            msg.fsm_event = "READY";            break;
    case GOAL_INSPECT:     msg.fsm_event = "GOAL_INSPECT";     break;
    case GOAL_CANCEL:      msg.fsm_event = "GOAL_CANCEL";      break;
    case GOAL_PREEMPT:     msg.fsm_event = "GOAL_PREEMPT";     break;
    case GOAL_PAUSE:       msg.fsm_event = "GOAL_PAUSE";       break;
    case GOAL_UNPAUSE:     msg.fsm_event = "GOAL_UNPAUSE";     break;
    case MOTION_SUCCESS:   msg.fsm_event = "MOTION_SUCCESS";   break;
    case MOTION_FAILED:    msg.fsm_event = "MOTION_FAILED";    break;
    case MOTION_UPDATE:
      msg.fsm_event = "MOTION_UPDATE";
      msg.fsm_subevent = m_fsm_subevent_;
      msg.fsm_substate = m_fsm_substate_;
      break;
    case DOCK_SUCCESS:     msg.fsm_event = "DOCK_SUCCESS";     break;
    case DOCK_FAILED:      msg.fsm_event = "DOCK_FAILED";      break;
    case DOCK_UPDATE:
      msg.fsm_event = "DOCK_UPDATE";
      msg.fsm_subevent = d_fsm_subevent_;
      msg.fsm_substate = d_fsm_substate_;
    break;
    case NEXT_INSPECT:     msg.fsm_event = "NEXT_INSPECT";  break;
    case INSPECT_FAILED:   msg.fsm_event = "INSPECT_FAILED";   break;
    case INSPECT_UPDATE:
      msg.fsm_event = "INSPECT_UPDATE";
      msg.fsm_subevent = "";
      msg.fsm_substate = i_fsm_substate_;
    break;
    case MANUAL_STATE_SET: msg.fsm_event = "MANUAL_STATE_SET"; break;
    }
    ROS_DEBUG_STREAM("Received event " << msg.fsm_event);
    // Debug state changes
    switch (state) {
    case STATE::INITIALIZING:
      msg.fsm_state = "INITIALIZING";                      break;
    case STATE::WAITING:
      msg.fsm_state = "WAITING";                           break;
    case STATE::INIT_INSPECTION:
      msg.fsm_state = "INIT_INSPECTION";                   break;
    case STATE::MOVING_TO_APPROACH_POSE:
      msg.fsm_state = "MOVING_TO_APPROACH_POSE";           break;
    case STATE::VISUAL_INSPECTION:
      msg.fsm_state = "VISUAL_INSPECTION";                 break;
    case STATE::RETURN_INSPECTION:
      msg.fsm_state = "RETURN_INSPECTION";                 break;
    }
    ROS_DEBUG_STREAM("State changed to " << msg.fsm_state);
    // Broadcast the docking state
    pub_state_.publish(msg);
    // Send the feedback if needed
    switch (state) {
    case STATE::INITIALIZING:
    case STATE::WAITING:
      break;
    default:
      {
        ROS_DEBUG_STREAM("InspectionFeedback ");
        isaac_msgs::InspectionFeedback feedback;
        feedback.state = msg;
        server_.SendFeedback(feedback);
      }
    }
  }

  // Preempt the current action with a new action
  void PreemptCallback() {
    NODELET_DEBUG_STREAM("PreemptCallback");
    return fsm_.Update(GOAL_PREEMPT);
  }

  // A Cancellation request arrives
  void CancelCallback() {
    NODELET_DEBUG_STREAM("CancelCallback");
    return fsm_.Update(GOAL_CANCEL);
  }

 private:
  ros::NodeHandle* nh_;
  ff_util::FSM fsm_;
  ff_util::FreeFlyerActionClient<ff_msgs::MotionAction> client_m_;
  ff_util::FreeFlyerActionClient<ff_msgs::DockAction> client_d_;
  ff_util::FreeFlyerActionClient<isaac_msgs::ImageInspectionAction> client_i_;
  ff_msgs::DockState dock_state_;
  ff_util::FreeFlyerActionServer<isaac_msgs::InspectionAction> server_;
  ff_util::ConfigServer cfg_;
  ros::Publisher pub_state_;
  ros::Publisher pub_guest_sci_;
  ros::Subscriber sub_sci_cam_;
  ros::ServiceServer server_set_state_;
  isaac_msgs::InspectionGoal goal_;
  int goal_counter_= 0;
  std::string m_fsm_subevent_, m_fsm_substate_;
  std::string d_fsm_subevent_, d_fsm_substate_;
  std::string i_fsm_substate_;
  isaac_msgs::InspectionResult result_;
  int motion_retry_number_= 0;
  int max_motion_retry_number_ = 0;
  // Flag to wait for sci camera
  bool sci_cam_req_ = false;
  bool ground_active_ = false;
  bool sim_mode_ = false;

  // Inspection library
  Inspection* inspection_;
};

PLUGINLIB_EXPORT_CLASS(inspection::InspectionNode, nodelet::Nodelet);

}  // namespace inspection

