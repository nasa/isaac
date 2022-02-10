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

// Command line flags
#include <gflags/gflags.h>
#include <gflags/gflags_completions.h>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Eigen>

// FSW includes
#include <ff_util/ff_names.h>
#include <isaac_util/isaac_names.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>

// Messages
#include <gazebo_msgs/SetModelState.h>

// TF2 support
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Boost
#include <boost/filesystem.hpp>

// C++ STL includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include <chrono>
#include <random>
#include <thread>

// Robot namespace
DEFINE_string(ns, "", "Robot namespace");
DEFINE_string(path_dataset, std::string(std::getenv("HOME")) + "/train_data", "Path to the saved datasets");
DEFINE_string(vent_poses, "resources/vent_poses", "File containing the vent poses");
DEFINE_string(other_poses, "resources/other_poses", "File containing the other non-vent poses");
DEFINE_double(robot_dist, 1.0, "Robot's distance to vent");
DEFINE_int32(train_pics_per_vent, 10, "Pictures taken per vent for train data");
DEFINE_int32(test_pics_per_vent, 3, "Pictures taken per vent for test data");

int count_ = 0;
std::string folder_, read_file_;
bool sock_ = false;
ros::ServiceClient client_;
ros::Publisher cmd_pub_;
ros::Subscriber sub_sci_cam_;

bool has_only_whitespace_or_comments(const std::string & str) {
  for (std::string::const_iterator it = str.begin(); it != str.end(); it++) {
    if (*it == '#') return true;  // No need to check further
    if (*it != ' ' && *it != '\t' && *it != '\n' && *it != '\r') return false;
  }
  return true;
}

void ChangePoses(tf2::Transform &vent_transform) {
  // construct a trivial random generator engine from a time-based seed:
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);

  // Convert vent pose to Euler
  tf2::Quaternion quat_vent;
  quat_vent = vent_transform.getRotation();
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat_vent).getRPY(roll, pitch, yaw);

  std::normal_distribution<double> robot_distribution_x(0.0, 0.02);
  std::normal_distribution<double> robot_distribution_y(0.0, 0.02);
  std::normal_distribution<double> robot_distribution_z(0.0, 0.02);
  std::normal_distribution<double> robot_distribution_roll(0.0, 0.03);
  std::normal_distribution<double> robot_distribution_pitch(0.0, 0.03);
  std::normal_distribution<double> robot_distribution_yaw(0.0, 0.03);

  // Change Astrobee pose
  gazebo_msgs::ModelState robotstate;
  robotstate.model_name = (std::string) "bsharp";
  robotstate.reference_frame = (std::string) "world";

  // Get robot distance to vent
  tf2::Vector3 robot_position = vent_transform * tf2::Vector3(0, 0, FLAGS_robot_dist);
  robotstate.pose.position.x = robot_position.x() + robot_distribution_x(generator);
  robotstate.pose.position.y = robot_position.y() + robot_distribution_y(generator);
  robotstate.pose.position.z = robot_position.z() + robot_distribution_z(generator);
  // std::cout << "Robot position " << robot_position.x() << " " << robot_position.y() <<
  //              " " << robot_position.z() << std::endl;

  tf2::Quaternion quat_robot;
  quat_robot.setRPY(0       + robot_distribution_roll(generator),
                    1.57075 + robot_distribution_pitch(generator),
                    0       + robot_distribution_yaw(generator));
  quat_robot = vent_transform.getRotation() * quat_robot;

  robotstate.pose.orientation.x = quat_robot.x();
  robotstate.pose.orientation.y = quat_robot.y();
  robotstate.pose.orientation.z = quat_robot.z();
  robotstate.pose.orientation.w = quat_robot.w();
  // std::cout << "Robot orientation " << roll << " " << pitch << " " << yaw << std::endl << std::endl;

  robotstate.twist.linear.x = 0.0;
  robotstate.twist.linear.y = 0.0;
  robotstate.twist.linear.z = 0.0;
  robotstate.twist.angular.x = 0.0;
  robotstate.twist.angular.y = 0.0;
  robotstate.twist.angular.z = 0.0;

  gazebo_msgs::SetModelState setrobotstate;
  setrobotstate.request.model_state = robotstate;
  client_.call(setrobotstate);

  // Change sock pose
  if (sock_) {
    std::normal_distribution<double> sock_distribution_x(0.0, 0.02);
    std::normal_distribution<double> sock_distribution_y(0.0, 0.02);
    std::normal_distribution<double> sock_distribution_z(0.0, 0.02);
    std::normal_distribution<double> sock_distribution_roll(0.0, 0.05);
    std::normal_distribution<double> sock_distribution_pitch(0.0, 0.05);
    std::normal_distribution<double> sock_distribution_yaw(0.0, 2.0);

    gazebo_msgs::ModelState sockstate;
    sockstate.model_name = (std::string) "object";
    sockstate.reference_frame = (std::string) "world";

    tf2::Vector3 sock_position = vent_transform.getOrigin();
    sockstate.pose.position.x = sock_position.x() + sock_distribution_x(generator);
    sockstate.pose.position.y = sock_position.y() + sock_distribution_y(generator);
    sockstate.pose.position.z = sock_position.z() + sock_distribution_z(generator);

    tf2::Quaternion quat_sock;
    quat_sock.setRPY(sock_distribution_roll(generator),
                     sock_distribution_pitch(generator),
                     sock_distribution_yaw(generator));
    quat_sock = vent_transform.getRotation() * quat_sock;

    sockstate.pose.orientation.x = quat_sock.x();
    sockstate.pose.orientation.y = quat_sock.y();
    sockstate.pose.orientation.z = quat_sock.z();
    sockstate.pose.orientation.w = quat_sock.w();

    sockstate.twist.linear.x = 0.0;
    sockstate.twist.linear.y = 0.0;
    sockstate.twist.linear.z = 0.0;
    sockstate.twist.angular.x = 0.0;
    sockstate.twist.angular.y = 0.0;
    sockstate.twist.angular.z = 0.0;

    gazebo_msgs::SetModelState setsockstate;
    setsockstate.request.model_state = sockstate;
    client_.call(setsockstate);
  }
  // std::cout << "Sending set state service... " << std::endl;
}

void PublishSciCamCommand(std::string const& cmdVal) {
  ff_msgs::CommandArg arg;
  std::vector<ff_msgs::CommandArg> cmd_args;

  // The command sends two strings. The first has the app name,
  // and the second the command value, encoded as json.

  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = "gov.nasa.arc.irg.astrobee.sci_cam_image";
  cmd_args.push_back(arg);

  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = "{\"name\": \"" + cmdVal + "\"}";
  cmd_args.push_back(arg);

  ff_msgs::CommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE;
  cmd.cmd_id = "train_data" + std::to_string(ros::Time::now().toSec());
  cmd.cmd_src = "guest science";
  cmd.cmd_origin = "guest science";
  cmd.args = cmd_args;

  // Allow image to stabilize
  ros::Duration(1.0).sleep();
  // Signal an imminent sci cam image
  cmd_pub_.publish(cmd);
  ros::Duration(1.0).sleep();
}

void SciCamCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
  // Analyse the received picture
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  try {
        cv_ptr->header = msg->header;
        cv_ptr->image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
  }
  catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  std::string filename = folder_ + std::string("/img")
                          + std::to_string(count_++) + std::string(".jpg");
  std::cout << "Writing "<< filename << std::endl;

  // Resize the input image and transform
  cv::Mat input_img = cv_ptr->image;
  cv::resize(input_img, input_img, cv::Size(244, 244), 0, 0, cv::INTER_CUBIC);

  ROS_ASSERT(cv::imwrite(filename, input_img));
}

int ReadFile() {
  std::ifstream ifs(read_file_.c_str());
  if (!ifs.is_open()) {
    std::cout << "Could not open file: " << FLAGS_vent_poses << std::endl;
    return -1;
  }

  std::string line;
  tf2::Transform vent_transform;
  tf2::Quaternion quat_vent;
  while (getline(ifs, line)) {
    if (has_only_whitespace_or_comments(line)) continue;

    std::istringstream is(line);
    double origin_x, origin_y, origin_z, euler_roll, euler_pitch, euler_yaw;
    if (!(is >> origin_x >> origin_y >> origin_z >> euler_roll >> euler_pitch >> euler_yaw)) {
      std::cout << "Ignoring invalid line: " << line  << std::endl;
      continue;
    }
    // Fill in vent pose
    vent_transform.setOrigin(tf2::Vector3(origin_x, origin_y, origin_z));
    quat_vent.setRPY(euler_roll,
                      euler_pitch,
                      euler_yaw);
    vent_transform.setRotation(quat_vent);

    // Test
    ChangePoses(vent_transform);
    PublishSciCamCommand("takeSinglePicture");
    ros::spinOnce();
  }
  return 0;
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Gather some data from the command
  google::SetUsageMessage("Usage: sci_cam_tool -cmd <cmd name>. "
                          "See below for allowed values for the -cmd option.");
  google::SetVersionString("0.1.0");
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Initialize a ros node
  ros::init(argc, argv, "get_train_data", ros::init_options::AnonymousName);

  // Create a node handle
  ros::NodeHandle nh(std::string("/") + FLAGS_ns);

  // Initialize command to change robot's position
  client_ = nh.serviceClient<gazebo_msgs::SetModelState>(
                                                  "/gazebo/set_model_state");
  // Initialize the command to take images
  cmd_pub_ = nh.advertise<ff_msgs::CommandStamped>(
                    TOPIC_COMMAND, 1);
  // Initialize image subscriber
  sub_sci_cam_ = nh.subscribe(TOPIC_HARDWARE_SCI_CAM + std::string("/compressed"),
                     100, SciCamCallback);

  // TRAIN DATA
  // ------------------------
  // No vent no sock-----------------
  sock_ = false;
  read_file_ = FLAGS_other_poses;

  folder_ = FLAGS_path_dataset + "/train/unknown";
  boost::filesystem::create_directories(folder_);

  for (int i = 0; i < FLAGS_train_pics_per_vent; ++i)
    ReadFile();

  // ------------------------
  // No obstable---------------------
  sock_ = false;
  read_file_ = FLAGS_vent_poses;
  folder_ = FLAGS_path_dataset + "/train/free";
  boost::filesystem::create_directories(folder_);

  for (int i = 0; i < FLAGS_train_pics_per_vent; ++i)
    ReadFile();

  // TEST DATA
  // ------------------------
  // No vent no sock-----------------
  sock_ = false;
  read_file_ = FLAGS_other_poses;
  folder_ = FLAGS_path_dataset + "/test/unknown";
  boost::filesystem::create_directories(folder_);

  for (int i = 0; i < FLAGS_test_pics_per_vent; ++i)
    ReadFile();

  // ------------------------
  // No obstable---------------------
  sock_ = false;
  read_file_ = FLAGS_vent_poses;
  folder_ = FLAGS_path_dataset + "/test/free";
  boost::filesystem::create_directories(folder_);

  for (int i = 0; i < FLAGS_test_pics_per_vent; ++i)
    ReadFile();


  // spawn sock////////////////////////////////////////////////////////////////
  std::cout << "Sending roslaunch command" << std::endl;
  FILE* pipe = popen("roslaunch isaac_gazebo spawn_object.launch spawn:=sock", "r");
  if (!pipe) {
      std::cerr << "Couldn't start command." << std::endl;
      return 0;
  }

  // TRAIN DATA
  // ------------------------
  // No vent with sock-----------------
  sock_ = true;
  read_file_ = FLAGS_other_poses;
  folder_ = FLAGS_path_dataset + "/train/unknown";
  boost::filesystem::create_directories(folder_);

  for (int i = 0; i < FLAGS_train_pics_per_vent; ++i)
    ReadFile();

  // ------------------------
  // Obstacle------------------------
  sock_ = true;
  read_file_ = FLAGS_vent_poses;
  folder_ = FLAGS_path_dataset + "/train/obstacle";
  boost::filesystem::create_directories(folder_);

  for (int i = 0; i < FLAGS_train_pics_per_vent; ++i)
    ReadFile();

  // TEST DATA
  // ------------------------
  // No vent with sock-----------------
  sock_ = true;
  read_file_ = FLAGS_other_poses;
  folder_ = FLAGS_path_dataset + "/test/unknown";
  boost::filesystem::create_directories(folder_);

  for (int i = 0; i < FLAGS_test_pics_per_vent; ++i)
    ReadFile();

  // ------------------------
  // Obstacle------------------------
  sock_ = true;
  read_file_ = FLAGS_vent_poses;
  folder_ = FLAGS_path_dataset + "/test/obstacle";
  boost::filesystem::create_directories(folder_);

  for (int i = 0; i < FLAGS_test_pics_per_vent; ++i)
    ReadFile();


  // Synchronous mode
  ros::spinOnce();
  std::cout << "Finished getting train and test data" << std::endl;

  // Make for great success
  return 0;
}
