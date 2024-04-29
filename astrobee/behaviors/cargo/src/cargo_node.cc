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

// TF2 support
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Static transform broadcaster
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

// Shared project includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_service.h>
#include <ff_util/ff_fsm.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>
#include <isaac_util/isaac_names.h>
#include <msg_conversions/msg_conversions.h>

// Software messages
#include <sensor_msgs/CompressedImage.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>

// Services
#include <ff_msgs/SetState.h>
#include <isaac_msgs/SetCargoAnomaly.h>

// Actions
#include <ff_msgs/ArmAction.h>
#include <ff_msgs/MotionAction.h>
#include <isaac_msgs/CargoAction.h>
#include <isaac_msgs/DetectionAction.h>

// Eigen for math
#include <Eigen/Dense>

typedef actionlib::SimpleActionServer<isaac_msgs::CargoAction> Server;

/**
 * \ingroup beh
 */
namespace cargo {

// Match the internal states and responses with the message definition
using FSM = ff_util::FSM;
using STATE = isaac_msgs::CargoState;
using RESPONSE = isaac_msgs::CargoResult;

/*
  This class provides the high-level logic that allows the freeflyer to
  perform inspection, detecting any anomaly. Please see the accompanying
  state diagram for a high level overview of this logic.

  Here are the rules of inspection:

    * Vent blocked: the freeflyer will approach the possibly obstructed
    vent and run image analysis to detect if the vent is obstructed.

    * Other: todo
*/
class CargoNode : public ff_util::FreeFlyerNodelet {
 public:
  // All possible events that can occur
  enum : FSM::Event {
    READY            = (1<<0),    // System is initialized
    GOAL_PICK        = (1<<1),    // Goal to start picking cargo
    GOAL_DROP        = (1<<2),    // Goal to start dropping cargo
    GOAL_CANCEL      = (1<<3),    // Cancel an existing goal
    GOAL_PREEMPT     = (1<<4),    // Preempt an existing goal
    GOAL_PAUSE       = (1<<5),    // Pause an existing goal
    GOAL_UNPAUSE     = (1<<6),    // Resume an existing goal
    MOTION_SUCCESS   = (1<<7),    // Mobility motion action success
    MOTION_FAILED    = (1<<8),    // Mobility motion action problem
    ARM_SUCCESS      = (1<<9),    // Arm motion action success
    ARM_FAILED       = (1<<10),   // Arm motion action problem
    DETECT_SUCCESS   = (1<<11),   // Detection AR tag success
    DETECT_FAILED    = (1<<12),   // Detection AR tag failed
    MANUAL_STATE_SET = (1<<13)    // Setting the state manually with service
  };

  // Positions that we might need to move to
  enum CargoPose {
    APPROACH_POSE,       // The starting poses of cargo pickup/drop
    BERTHING_POSE,       // The completion pose of cargo pickup/drop
    GOAL_APPROACH_POSE,
    GOAL_BERTHING_POSE
  };

  CargoNode() : ff_util::FreeFlyerNodelet(NODE_CARGO, false),
    fsm_(STATE::INITIALIZING, std::bind(&CargoNode::UpdateCallback,
      this, std::placeholders::_1, std::placeholders::_2)) {
    // Add the state transition lambda functions - refer to the FSM diagram
    // [0]
    fsm_.Add(STATE::INITIALIZING,
      READY, [this](FSM::Event const& event) -> FSM::State {
        return STATE::WAITING;
      });
    // [1]
    fsm_.Add(STATE::WAITING,
      GOAL_PICK, [this](FSM::Event const& event) -> FSM::State {
        // Move to approach pose
        if (!Move(APPROACH_POSE)) {
          Result(RESPONSE::MOTION_FAILED);
          return STATE::WAITING;
        }
        return STATE::PICK_MOVE_APPROACH;
      });
    // [2]
    fsm_.Add(STATE::PICK_MOVE_APPROACH,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Arm(ff_msgs::ArmGoal::ARM_DEPLOY);
        return STATE::PICK_ARM_DEPLOY;
      });
    // [3]
    fsm_.Add(STATE::PICK_ARM_DEPLOY,
      ARM_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Arm(ff_msgs::ArmGoal::GRIPPER_OPEN);
        return STATE::PICK_ARM_OPEN;
      });
    // [3]
    fsm_.Add(STATE::PICK_ARM_OPEN,
      ARM_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        // Move to the handle
        if (!Move(BERTHING_POSE)) {
          Result(RESPONSE::MOTION_FAILED);
          return STATE::WAITING;
        }
        return STATE::PICK_MOVE_COMPLETE;
      });
    // [4]
    fsm_.Add(STATE::PICK_MOVE_COMPLETE,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        // Move to the handle
        Arm(ff_msgs::ArmGoal::GRIPPER_CLOSE);
        return STATE::PICK_ARM_CLOSE;
      });
    // [5]
    fsm_.Add(STATE::PICK_ARM_CLOSE,
      ARM_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        // Move back to the approach pose
        if (!Move(APPROACH_POSE)) {
          Result(RESPONSE::MOTION_FAILED);
          return STATE::HOLDING;
        }
        return STATE::PICK_MOVE_FINAL;
      });
    // [5]
    fsm_.Add(STATE::PICK_MOVE_FINAL,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        // After successful pickup
        Result(RESPONSE::SUCCESS, "Holding the cargo");
        return STATE::HOLDING;
      });
    // [6]
    fsm_.Add(STATE::HOLDING,
      GOAL_DROP, [this](FSM::Event const& event) -> FSM::State {
        // Move to the approach position
        ROS_ERROR_STREAM("GOT THE GOAL IN STATE MACHINE");
        if (!Move(GOAL_APPROACH_POSE)) {
          Result(RESPONSE::MOTION_FAILED);
          return STATE::HOLDING;
        }
        return STATE::DROP_MOVE_APPROACH;
      });
    // [7]
    fsm_.Add(STATE::DROP_MOVE_APPROACH,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        // Introduce anomaly
        if (anomaly_ == isaac_msgs::SetCargoAnomaly::Request::BERTH_FULL) {
          anomaly_ = isaac_msgs::SetCargoAnomaly::Request::NO_ANOMALY;
          Result(RESPONSE::BERTH_FULL);
          return STATE::HOLDING;
        }
        // Move to the drop position
        if (!Move(GOAL_BERTHING_POSE)) {
          Result(RESPONSE::MOTION_FAILED);
          return STATE::HOLDING;
        }
        return STATE::DROP_MOVE_COMPLETE;
      });
    // [7]
    fsm_.Add(STATE::DROP_MOVE_COMPLETE,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        // Open gripper to release bag
        Arm(ff_msgs::ArmGoal::GRIPPER_OPEN);
        return STATE::DROP_ARM_OPEN;
      });
    // [8]
    fsm_.Add(STATE::DROP_ARM_OPEN,
      ARM_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        // Move away from the bag
        if (!Move(GOAL_APPROACH_POSE)) {
          Result(RESPONSE::MOTION_FAILED);
          return STATE::WAITING;
        }
        return STATE::DROP_MOVE_AWAY;
      });
    // [9]
    fsm_.Add(STATE::DROP_MOVE_AWAY,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        // Stow the arm
        Arm(ff_msgs::ArmGoal::ARM_STOW);
        return STATE::DROP_ARM_STOW;
      });
    // [10]
    fsm_.Add(STATE::DROP_ARM_STOW,
      ARM_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        // Fill in drop position
        geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
          "world", "cargo_goal/body", ros::Time::now(), ros::Duration(3));
        // Copy the transform
        result_.final_cargo_pose.pose.position.x = tf.transform.translation.x;
        result_.final_cargo_pose.pose.position.y = tf.transform.translation.y;
        result_.final_cargo_pose.pose.position.z = tf.transform.translation.z;
        result_.final_cargo_pose.pose.orientation = tf.transform.rotation;
        // After successful drop
        Result(RESPONSE::SUCCESS, "Stowed the cargo");
        return STATE::WAITING;
      });
    // [11]
    fsm_.Add(MOTION_FAILED | ARM_FAILED | DETECT_FAILED,
      [this](FSM::State const& state, FSM::Event const& event) -> FSM::State {
        switch (event) {
        case MOTION_FAILED:
          Result(RESPONSE::MOTION_FAILED, err_msg_);
          break;
        case ARM_FAILED:
          Result(RESPONSE::ARM_FAILED, err_msg_);
          break;
        case DETECT_FAILED:
          Result(RESPONSE::DETECT_FAILED);
          break;
        }
        switch (state) {
        // Motion in progress
        case STATE::PICK_MOVE_APPROACH:
        case STATE::PICK_MOVE_COMPLETE:
        case STATE::DROP_MOVE_AWAY:
          return STATE::WAITING;
        // Motion in progress
        case STATE::PICK_MOVE_FINAL:
        case STATE::DROP_MOVE_APPROACH:
          return STATE::HOLDING;
        // Arm movement in progress
        case STATE::PICK_ARM_DEPLOY:
        case STATE::PICK_ARM_OPEN:
        case STATE::PICK_ARM_CLOSE:
        case STATE::DROP_ARM_OPEN:
        case STATE::DROP_ARM_STOW:
          return STATE::WAITING;
        // Detect in progress
        case STATE::PICK_DETECT:
          return STATE::WAITING;
        }
        return STATE::WAITING;
      });
    //////////////////////////////////////////////////////
    // CATCH-ALL FOR CANCELLATIONS / PREEMPTION / PAUSE //
    //////////////////////////////////////////////////////
    fsm_.Add(GOAL_CANCEL | GOAL_PREEMPT | GOAL_PAUSE,
      [this](FSM::State const& state, FSM::Event const& event) -> FSM::State {
        switch (state) {
        // Motion in progress
        case STATE::PICK_MOVE_APPROACH:
        case STATE::PICK_MOVE_COMPLETE:
        case STATE::PICK_MOVE_FINAL:
        case STATE::DROP_MOVE_AWAY:
          client_m_.CancelGoal();
          return STATE::WAITING;
        // Motion in progress
        case STATE::DROP_MOVE_APPROACH:
          client_m_.CancelGoal();
          return STATE::HOLDING;
        // Arm movement
        case STATE::PICK_ARM_DEPLOY:
        case STATE::PICK_ARM_OPEN:
        case STATE::PICK_ARM_CLOSE:
        case STATE::DROP_ARM_OPEN:
        case STATE::DROP_ARM_STOW:
          client_a_.CancelGoal();
          return STATE::WAITING;
        case STATE::PICK_DETECT:
          client_d_.CancelGoal();
          return STATE::WAITING;
        case STATE::HOLDING:
          return STATE::HOLDING;
        case STATE::WAITING:
          return STATE::WAITING;
        }
        return STATE::WAITING;
      });
  }

  ~CargoNode() {}

 protected:
  void Initialize(ros::NodeHandle* nh) {
    nh_ = nh;
    // Set the config path to ISAAC
    char *path = getenv("CUSTOM_CONFIG_DIR");
    if (path != NULL)
      cfg_.SetPath(path);
    // Grab some configuration parameters for this node from the LUA config reader
    cfg_.Initialize(GetPrivateHandle(), "behaviors/cargo.config");
    if (!cfg_.Listen(boost::bind(
      &CargoNode::ReconfigureCallback, this, _1)))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                         "Could not load config");

    // Create a transform buffer to listen for transforms
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(tf_buffer_));

    // Publish the cargo state as a latched topic
    pub_state_ = nh->advertise<isaac_msgs::CargoState>(
      TOPIC_BEHAVIORS_CARGO_STATE, 1, true);

    // Allow the state to be manually set
    server_set_state_ = nh->advertiseService(SERVICE_BEHAVIORS_CARGO_SET_STATE,
      &CargoNode::SetStateCallback, this);

    // Allow the state to be manually set
    server_set_anomaly_ = nh->advertiseService(SERVICE_BEHAVIORS_CARGO_SET_ANOMALY,
      &CargoNode::SetAnomalyCallback, this);

    // Setup move client action
    client_m_.SetConnectedTimeout(cfg_.Get<double>("timeout_motion_connected"));
    client_m_.SetActiveTimeout(cfg_.Get<double>("timeout_motion_active"));
    client_m_.SetResponseTimeout(cfg_.Get<double>("timeout_motion_response"));
    client_m_.SetFeedbackCallback(std::bind(&CargoNode::MFeedbackCallback,
      this, std::placeholders::_1));
    client_m_.SetResultCallback(std::bind(&CargoNode::MResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_m_.SetConnectedCallback(std::bind(
      &CargoNode::ConnectedCallback, this));
    client_m_.Create(nh, ACTION_MOBILITY_MOTION);

    // Setup arm client action
    client_a_.SetConnectedTimeout(cfg_.Get<double>("timeout_arm_connected"));
    client_a_.SetActiveTimeout(cfg_.Get<double>("timeout_arm_active"));
    client_a_.SetResponseTimeout(cfg_.Get<double>("timeout_arm_response"));
    client_a_.SetFeedbackCallback(std::bind(&CargoNode::AFeedbackCallback,
      this, std::placeholders::_1));
    client_a_.SetResultCallback(std::bind(&CargoNode::AResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_a_.SetConnectedCallback(std::bind(
      &CargoNode::ConnectedCallback, this));
    client_a_.Create(nh, ACTION_BEHAVIORS_ARM);

    // Setup image analysis client action
    // client_i_.SetConnectedTimeout(cfg_.Get<double>("timeout_detection_connected"));
    // client_i_.SetActiveTimeout(cfg_.Get<double>("timeout_detection_active"));
    client_d_.SetResponseTimeout(cfg_.Get<double>("timeout_detection_response"));
    client_d_.SetFeedbackCallback(std::bind(&CargoNode::DFeedbackCallback,
      this, std::placeholders::_1));
    client_d_.SetResultCallback(std::bind(&CargoNode::DResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_d_.SetConnectedCallback(std::bind(
      &CargoNode::GroundConnectedCallback, this));
    client_d_.Create(nh, ACTION_AR_DETECTION);

    // Setup the execute action
    server_.SetGoalCallback(std::bind(
      &CargoNode::GoalCallback, this, std::placeholders::_1));
    server_.SetPreemptCallback(std::bind(
      &CargoNode::PreemptCallback, this));
    server_.SetCancelCallback(std::bind(
      &CargoNode::CancelCallback, this));
    server_.Create(nh, ACTION_BEHAVIORS_CARGO);
  }

  // Ensure all clients are connected
  void ConnectedCallback() {
    NODELET_DEBUG_STREAM("ConnectedCallback()");
    if (!client_m_.IsConnected()) return;       // Move action
    if (!client_a_.IsConnected()) return;       // Arm action
    fsm_.Update(READY);                         // Ready!
  }

  void GroundConnectedCallback() {
    NODELET_DEBUG_STREAM("GroundConnectedCallback()");
    if (!client_d_.IsConnected()) return;       // Detection action
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

  // Service callback to manually change the state
  bool SetAnomalyCallback(isaac_msgs::SetCargoAnomaly::Request& req,
                        isaac_msgs::SetCargoAnomaly::Response& res) {
    anomaly_ = req.anomaly;
    res.success = true;
    return true;
  }

  // Send a move command
  bool Move(CargoPose cargo_pose) {
    // Create a new motion goal
    ff_msgs::MotionGoal goal;
    goal.command = ff_msgs::MotionGoal::MOVE;
    goal.flight_mode = ff_msgs::MotionGoal::NOMINAL;

    // Package up the desired end pose
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    switch (cargo_pose) {
      case APPROACH_POSE:
      msg.header.frame_id = "cargo/approach";
      goal.states.push_back(msg);
        break;
      case BERTHING_POSE:
      msg.header.frame_id = "cargo/approach";
      goal.states.push_back(msg);
      msg.header.frame_id = "cargo/complete";
      goal.states.push_back(msg);
        break;
      case GOAL_APPROACH_POSE:
      msg.header.frame_id = "cargo_goal/approach";
      goal.states.push_back(msg);
        break;
      case GOAL_BERTHING_POSE:
      msg.header.frame_id = "cargo_goal/approach";
      msg.header.frame_id = "cargo_goal/complete";
      goal.states.push_back(msg);
        break;
        break;
    }

    // Iterate over all poses in action, finding the location of each
    for (auto & pose : goal.states) {
      try {
        // Look up the world -> berth transform
        geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
          "world", pose.header.frame_id, ros::Time::now(), ros::Duration(3));
        // Copy the transform
        pose.pose.position.x = tf.transform.translation.x;
        pose.pose.position.y = tf.transform.translation.y;
        pose.pose.position.z = tf.transform.translation.z;
        pose.pose.orientation = tf.transform.rotation;
      } catch (tf2::TransformException &ex) {
        ROS_ERROR_STREAM("Transform failed" << ex.what());
        return false;
      }
    }

    // Load parameters from config file
    std::string planner = cfg_.Get<std::string>("planner");
    bool coll_check = cfg_.Get<bool>("enable_collision_checking");
    bool faceforward = cfg_.Get<bool>("enable_faceforward");

    bool replanning = cfg_.Get<bool>("enable_replanning");
    int replanning_attempts = cfg_.Get<int>("max_replanning_attempts");
    bool validation = cfg_.Get<bool>("enable_validation");
    bool boostrapping = cfg_.Get<bool>("enable_bootstrapping");
    bool immediate = cfg_.Get<bool>("enable_immediate");
    bool timesync = cfg_.Get<bool>("enable_timesync");
    double desired_vel = cfg_.Get<double>("desired_vel");
    double desired_accel = cfg_.Get<double>("desired_accel");
    double desired_omega = cfg_.Get<double>("desired_omega");
    double desired_alpha = cfg_.Get<double>("desired_alpha");

    // Reconfigure the choreographer
    ff_util::ConfigClient choreographer_cfg(GetPlatformHandle(), NODE_CHOREOGRAPHER);
    choreographer_cfg.Set<std::string>("planner", planner);
    choreographer_cfg.Set<bool>("enable_collision_checking", coll_check);
    choreographer_cfg.Set<bool>("enable_faceforward", faceforward);
    choreographer_cfg.Set<bool>("enable_replanning", replanning);
    choreographer_cfg.Set<int>("max_replanning_attempts", replanning_attempts);
    choreographer_cfg.Set<bool>("enable_validation", validation);
    choreographer_cfg.Set<bool>("enable_bootstrapping", boostrapping);
    choreographer_cfg.Set<bool>("enable_immediate", immediate);
    choreographer_cfg.Set<bool>("enable_timesync", timesync);
    choreographer_cfg.Set<double>("desired_vel", desired_vel);
    choreographer_cfg.Set<double>("desired_accel", desired_accel);
    choreographer_cfg.Set<double>("desired_omega", desired_omega);
    choreographer_cfg.Set<double>("desired_alpha", desired_alpha);
    if (!choreographer_cfg.Reconfigure()) {
      NODELET_ERROR_STREAM("Failed to reconfigure choreographer");
      return false;
    }
    // Send the goal to the mobility subsystem
    return client_m_.SendGoal(goal);
  }

  // Ignore the move feedback, for now
  void MFeedbackCallback(ff_msgs::MotionFeedbackConstPtr const& feedback) {
  }

  // Result of a move action
  void MResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    ff_msgs::MotionResultConstPtr const& result) {
    switch (result_code) {
      case ff_util::FreeFlyerActionState::SUCCESS:
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
          // Try to find an alternate inspection position
          break;
      }
      ROS_ERROR_STREAM("Motion failed result error: " << result->response);
    } else {
      ROS_ERROR_STREAM("Invalid result received");
    }

    err_msg_ = "Move Code" + std::to_string(result->response) + ": (" + result->fsm_result + ")";
    return fsm_.Update(MOTION_FAILED);
  }

  // ARM ACTION CLIENT

  // Send a move command to the arm.
  bool Arm(uint8_t command) {
    ff_msgs::ArmGoal goal;
    goal.command = command;
    return client_a_.SendGoal(goal);
  }

  // Ignore the move feedback, for now
  void AFeedbackCallback(ff_msgs::ArmFeedbackConstPtr const& feedback) {
  }

  // Result of a move action
  void AResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
  ff_msgs::ArmResultConstPtr const& result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      return fsm_.Update(ARM_SUCCESS);
    default:
      err_msg_ = "Arm Code " + std::to_string(result->response) + ": (" + result->fsm_result + ")";
      return fsm_.Update(ARM_FAILED);
    }
  }

  // DETECTION ACTION CLIENT

  bool Detect() {
    // // Activate image anomaly detector if there are gound communications
    // if (goal_.command == isaac_msgs::InspectionGoal::ANOMALY && ground_active_) {
    //   // Send goal
    //   isaac_msgs::ImageInspectionGoal goal;
    //   goal.type = isaac_msgs::ImageInspectionGoal::ANOMALY;
    //   client_i_.SendGoal(goal);
    //   ROS_ERROR_STREAM("sent image inspection goal");
    // }

    // // Take picture
    // ff_msgs::CommandArg arg;
    // std::vector<ff_msgs::CommandArg> cmd_args;

    // // The command sends two strings. The first has the app name,
    // // and the second the command value, encoded as json.

    // arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
    // arg.s = "gov.nasa.arc.irg.astrobee.sci_cam_image";
    // cmd_args.push_back(arg);

    // arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
    // arg.s = "{\"name\": \"takeSinglePicture\"}";
    // cmd_args.push_back(arg);

    // ff_msgs::CommandStamped cmd;
    // cmd.header.stamp = ros::Time::now();
    // cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE;
    // cmd.cmd_id = "inspection" + std::to_string(ros::Time::now().toSec());
    // cmd.cmd_src = "guest science";
    // cmd.cmd_origin = "guest science";
    // cmd.args = cmd_args;

    // // Allow image to stabilize
    // ros::Duration(2.0).sleep();

    // // Signal an imminent sci cam image
    // sci_cam_req_ = true;
    // pub_guest_sci_.publish(cmd);

    // // Timer for the sci cam camera
    // ros::Timer sci_cam_timeout_ = nh_->createTimer(ros::Duration(5), &CargoNode::SciCamTimeout, this, true, false);
    return true;
  }

  // Feedback of an action
  void DFeedbackCallback(isaac_msgs::DetectionFeedbackConstPtr const& feedback) {
    ROS_ERROR_STREAM("DFeedbackCallback()");
  }

  // Result of an action
  void DResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    isaac_msgs::DetectionResultConstPtr const& result) {
    ROS_ERROR_STREAM("DResultCallback()");
  }

  // CARGO ACTION SERVER

  // A new arm action has been called
  void GoalCallback(isaac_msgs::CargoGoalConstPtr const& goal) {
    // Save new goal
    goal_ = *goal;
        ROS_ERROR_STREAM("GOT THE GOAL IN GoalCallback");

    // Identify command
    switch (goal_.command) {
    // Vent command
    case isaac_msgs::CargoGoal::PICK:
    {
      NODELET_DEBUG("Received Goal PICK");
      cargo_id_ = goal_.cargo_id;
      geometry_msgs::TransformStamped tf;
      // Trasform from cargo/body to cargo/approach
      tf.header.stamp = ros::Time::now();
      tf.header.frame_id = std::string("world");
      tf.child_frame_id = std::string("cargo/body");
      tf.transform.translation =
        msg_conversions::eigen_to_ros_vector(Eigen::Vector3d(
                                              goal_.pose.pose.position.x,
                                              goal_.pose.pose.position.y,
                                              goal_.pose.pose.position.z));
      tf.transform.rotation =
        msg_conversions::eigen_to_ros_quat(Eigen::Quaterniond(
                                              goal_.pose.pose.orientation.w,
                                              goal_.pose.pose.orientation.x,
                                              goal_.pose.pose.orientation.y,
                                              goal_.pose.pose.orientation.z));
      bc_.sendTransform(tf);
      // Wait for transform to update
      ros::Duration(2.0).sleep();

      return fsm_.Update(GOAL_PICK);
      break;
    }
    // Geometry command
    case isaac_msgs::CargoGoal::DROP:
    {
      NODELET_DEBUG("Received Goal DROP");

      geometry_msgs::TransformStamped tf;
      // Trasform from cargo/body to cargo/approach
      tf.header.stamp = ros::Time::now();
      tf.header.frame_id = std::string("world");
      tf.child_frame_id = std::string("cargo_goal/berth");
      tf.transform.translation =
        msg_conversions::eigen_to_ros_vector(Eigen::Vector3d(
                                              goal_.pose.pose.position.x,
                                              goal_.pose.pose.position.y,
                                              goal_.pose.pose.position.z));
      tf.transform.rotation =
        msg_conversions::eigen_to_ros_quat(Eigen::Quaterniond(
                                              goal_.pose.pose.orientation.w,
                                              goal_.pose.pose.orientation.x,
                                              goal_.pose.pose.orientation.y,
                                              goal_.pose.pose.orientation.z));
      bc_.sendTransform(tf);
      // Wait for transform to update
      ros::Duration(2.0).sleep();

      return fsm_.Update(GOAL_DROP);
      break;
    }
    // Invalid command
    default:
    {
      isaac_msgs::CargoResult result;
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
    if ( fsm_.GetState() == STATE::WAITING)
      return cfg_.Reconfigure(config);
    return false;
  }

  // When the FSM state changes we get a callback here, so that we
  // can choose to do various things.
  void UpdateCallback(FSM::State const& state, FSM::Event const& event) {
    // Debug events
    isaac_msgs::CargoState msg;
    msg.header.frame_id = GetPlatform();
    msg.header.stamp = ros::Time::now();
    msg.state = state;
    msg.cargo_id = cargo_id_;
    // Debug events
    switch (event) {
    case READY:            msg.fsm_event = "READY";            break;
    case GOAL_PICK:        msg.fsm_event = "GOAL_PICK";        break;
    case GOAL_DROP:        msg.fsm_event = "GOAL_DROP";        break;
    case GOAL_CANCEL:      msg.fsm_event = "GOAL_CANCEL";      break;
    case GOAL_PREEMPT:     msg.fsm_event = "GOAL_PREEMPT";     break;
    case GOAL_PAUSE:       msg.fsm_event = "GOAL_PAUSE";       break;
    case GOAL_UNPAUSE:     msg.fsm_event = "GOAL_UNPAUSE";     break;
    case MOTION_SUCCESS:   msg.fsm_event = "MOTION_SUCCESS";   break;
    case MOTION_FAILED:    msg.fsm_event = "MOTION_FAILED";    break;
    case ARM_SUCCESS:      msg.fsm_event = "ARM_SUCCESS";      break;
    case ARM_FAILED:       msg.fsm_event = "ARM_FAILED";       break;
    case DETECT_SUCCESS:   msg.fsm_event = "DETECT_SUCCESS";   break;
    case DETECT_FAILED:    msg.fsm_event = "DETECT_FAILED";    break;
    case MANUAL_STATE_SET: msg.fsm_event = "MANUAL_STATE_SET"; break;
    }
    ROS_DEBUG_STREAM("Received event " << msg.fsm_event);
    // Debug state changes
    switch (state) {
    case STATE::INITIALIZING:
      msg.fsm_state = "INITIALIZING";                      break;
    case STATE::WAITING:
      msg.fsm_state = "WAITING";                           break;
    case STATE::HOLDING:
      msg.fsm_state = "HOLDING";                           break;
    case STATE::PICK_MOVE_APPROACH:
      msg.fsm_state = "PICK_MOVE_APPROACH";                break;
    case STATE::PICK_DETECT:
      msg.fsm_state = "PICK_DETECT";                       break;
    case STATE::PICK_ARM_DEPLOY:
      msg.fsm_state = "PICK_ARM_DEPLOY";                   break;
    case STATE::PICK_ARM_OPEN:
      msg.fsm_state = "PICK_ARM_OPEN";                     break;
    case STATE::PICK_MOVE_COMPLETE:
      msg.fsm_state = "PICK_MOVE_COMPLETE";                break;
    case STATE::PICK_ARM_CLOSE:
      msg.fsm_state = "PICK_ARM_CLOSE";                    break;
    case STATE::PICK_MOVE_FINAL:
      msg.fsm_state = "PICK_MOVE_FINAL";                   break;
    case STATE::DROP_MOVE_APPROACH:
      msg.fsm_state = "DROP_MOVE_APPROACH";                break;
    case STATE::DROP_MOVE_COMPLETE:
      msg.fsm_state = "DROP_MOVE_COMPLETE";                break;
    case STATE::DROP_ARM_OPEN:
      msg.fsm_state = "DROP_ARM_OPEN";                     break;
    case STATE::DROP_MOVE_AWAY:
      msg.fsm_state = "DROP_MOVE_AWAY";                    break;
    case STATE::DROP_ARM_STOW:
      msg.fsm_state = "DROP_ARM_STOW";                     break;
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
        ROS_DEBUG_STREAM("CargoFeedback ");
        isaac_msgs::CargoFeedback feedback;
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
  ff_util::FreeFlyerActionClient<ff_msgs::ArmAction> client_a_;
  ff_util::FreeFlyerActionClient<isaac_msgs::DetectionAction> client_d_;
  // ff_msgs::DockState dock_state_;
  ff_util::FreeFlyerActionServer<isaac_msgs::CargoAction> server_;
  ff_util::ConfigServer cfg_;
  tf2_ros::StaticTransformBroadcaster bc_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  ros::Publisher pub_state_;
  ros::Publisher pub_guest_sci_;
  ros::Subscriber sub_sci_cam_;
  ros::ServiceServer server_set_state_;
  ros::ServiceServer server_set_anomaly_;
  isaac_msgs::CargoGoal goal_;
  std::string cargo_id_;
  isaac_msgs::CargoResult result_;
  std::string err_msg_;

  // Flag to wait for sci camera
  bool sci_cam_req_ = false;
  bool ground_active_ = false;
  bool image_ack_ = false;

  // Anomaly
  int anomaly_ = isaac_msgs::SetCargoAnomaly::Request::NO_ANOMALY;

  // Inspection library
  // Inspection* inspection_;
};

PLUGINLIB_EXPORT_CLASS(cargo::CargoNode, nodelet::Nodelet);

}  // namespace cargo

