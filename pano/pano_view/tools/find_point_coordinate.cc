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


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// Import messahes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ff_msgs/EkfState.h>


// Shared project includes
#include <inspection/camera_projection.h>

// json file reader lib
#include <jsoncpp/json/allocator.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/value.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

// Parameters
DEFINE_string(camera, "sci_cam", "Camera name.");
DEFINE_string(bag_name, "", "Bagname where image can be found. Make sure it has haz cam and ground truth");
DEFINE_string(json_config, "", "json file with configure data.");

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function 
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M> {
 public:
  void newMessage(const boost::shared_ptr<M const>& msg) { this->signalMessage(msg); }
};

// Callback for synchronized messages
void callback(const sensor_msgs::CameraInfo::ConstPtr& image_info, const sensor_msgs::PointCloud2::ConstPtr& depth_cam,
              const ff_msgs::EkfState::ConstPtr& ground_truth) {
  ROS_ERROR("SYNC CALLBACK");
}

void extractSyncTopicsBag(const std::string bagname, const std::string image_info_topic,
                          const std::string depth_cam_topic, const std::string ground_truth_topic) {
  ROS_ERROR("extractSyncTopicsBag");
  // Read ground truth from bag
  rosbag::Bag bag;
  ROS_ERROR_STREAM("bag" << bagname);
  bag.open(FLAGS_bag_name, rosbag::bagmode::Read);
  ROS_ERROR_STREAM("bag read" << bagname);

  // Set up fake subscribers to capture topics
  BagSubscriber<sensor_msgs::CameraInfo> image_info_sub;
  BagSubscriber<sensor_msgs::PointCloud2> depth_cam_sub;
  BagSubscriber<ff_msgs::EkfState> ground_truth_sub;

  // Use time synchronizer to make sure we get properly synchronized images
  message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, ff_msgs::EkfState> sync(
    image_info_sub, depth_cam_sub, ground_truth_sub, 25);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  std::vector<std::string> topics;
  topics.push_back(image_info_topic);
  topics.push_back(depth_cam_topic);
  topics.push_back(ground_truth_topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH(rosbag::MessageInstance const m, view) {
    // Only relevant image timestamp
    if (m.getTopic() == image_info_topic || ("/" + m.getTopic() == image_info_topic)) {
      sensor_msgs::CameraInfo::ConstPtr image_info = m.instantiate<sensor_msgs::CameraInfo>();
      if (image_info != NULL)
        image_info_sub.newMessage(image_info);
    }
    // Read all depth cam images
    if (m.getTopic() == depth_cam_topic || ("/" + m.getTopic() == depth_cam_topic)) {
      sensor_msgs::PointCloud2::ConstPtr depth_cam = m.instantiate<sensor_msgs::PointCloud2>();
      if (depth_cam != NULL)
        depth_cam_sub.newMessage(depth_cam);
    }
    // Read all ground truth data
    if (m.getTopic() == ground_truth_topic || ("/" + m.getTopic() == ground_truth_topic)) {
      ff_msgs::EkfState::ConstPtr ground_truth = m.instantiate<ff_msgs::EkfState>();
      if (ground_truth != NULL)
        ground_truth_sub.newMessage(ground_truth);
    }
  }

  bag.close();
}

int main(int argc, char** argv) {
  // Initialize a ros node
  ros::init(argc, argv, "find_point_coordinate", ros::init_options::AnonymousName);
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun pano_view find_point_coordinate <opts>");
  google::SetVersionString("0.1.0");
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Read parameters from config file
  std::ifstream input_file(FLAGS_json_config);
  if (!input_file.is_open()) {
    std::cerr << "Failed to open file: " << FLAGS_json_config << std::endl;
    return 1;
  }
  Json::Reader reader;
  Json::Value json;
  bool success = reader.parse(input_file, json);
  if (!success) {
    std::cerr << "Failed to parse JSON data: " << reader.getFormattedErrorMessages() << std::endl;
    return 1;
  }

  double timestamp = json["timestamp"].asDouble();
  std::string camera_name = json["camera"].asString();
  int coord_x = json["coord"]["x"].asInt();
  int coord_y = json["coord"]["y"].asInt();


  // Initialize camera view
  inspection::CameraView camera(camera_name);


  // Extract topics from bagfile
  std::string depth_cam_topic = "/hw/depth_haz/points";
  std::string image_info_topic = "/hw/cam_sci_info";
  std::string ground_truth_topic = "/ground_truth/gnc/ekf";
  extractSyncTopicsBag(FLAGS_bag_name, image_info_topic, depth_cam_topic, ground_truth_topic);


  // Figure out the vector to the target based on camera parameters


  // Target position based on haz cam measurements


  // Target position based on 3D mesh model
  // if(intersectRayMesh(FLAGS_bag_name, )) {
  //   ROS_ERROR("SYNC CALLBACK");
  // } else {
  //   ROS_ERROR("SYNC CALLBACK");
  // }

  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
