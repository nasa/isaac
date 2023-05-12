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

#ifndef DENSE_MAP_ROS_UTILS_H_
#define DENSE_MAP_ROS_UTILS_H_

// ROS includes
#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dense_map_utils.h>

#include <map>
#include <string>
#include <vector>

// Forward declaration
namespace pcl {
  class PointXYZ;
  template<class T>
  class PointCloud;
}

namespace dense_map {

// Publish a given pose
void PublishTF(const Eigen::Affine3d& publish_tf, std::string const& parent_frame, std::string const& child_frame,
               ros::Time const& timestamp, tf2_ros::TransformBroadcaster& tf_publisher);

void readBagPoses(std::string const& bag_file, std::string const& topic, StampedPoseStorage& poses);

void readBagImageTimestamps(std::string const& bag_file, std::string const& topic, std::vector<double>& timestamps);

// Given a bag view, for each topic in the view read the vector of
// messages for that topic, sorted by message header timestamp. Only
// the following sensor types are supported: sensor_msgs::Image,
// sensor_msgs::CompressedImage, and sensor_msgs::PointCloud2.
void indexMessages(rosbag::View& view,  // view can't be made const
                   std::map<std::string, std::vector<rosbag::MessageInstance>>& bag_map);

void readExifFromBag(std::vector<rosbag::MessageInstance> const& bag_msgs, std::map<double, std::vector<double>>& exif);

// Find an image at the given timestamp or right after it. We assume
// that during repeated calls to this function we always travel
// forward in time, and we keep track of where we are in the bag using
// the variable bag_pos that we update as we go.
bool lookupImage(double desired_time, std::vector<rosbag::MessageInstance> const& bag_msgs, bool save_grayscale,
                 cv::Mat& image, int& bag_pos, double& found_time, std::string image_dir = "");

// Find the closest depth cloud to given timestamp. Return an empty
// cloud if one cannot be found closer in time than max_time_diff.
// Store it as a cv::Mat of vec3f values. We assume that during
// repeated calls to this function we always travel forward in time,
// and we keep track of where we are in the bag using the variable
// bag_pos that we update as we go.
bool lookupCloud(double desired_time, std::vector<rosbag::MessageInstance> const& bag_msgs,
                 double max_time_diff, cv::Mat& cloud, int& bag_pos, double& found_time);

// A wrapper around a function in pcl_ros/point_cloud.h to avoid
// including that header all over the place as it creates an annoying
// warning.
void msgToPcl(sensor_msgs::PointCloud2::ConstPtr pc_msg, pcl::PointCloud<pcl::PointXYZ> & pc);

// Read the list of topics in a bag while avoiding repetitions
void readTopicsInBag(std::string const& bag_file, std::vector<std::string>& topics);

// A small struct in which to store an opened ROS bag and the vector of its messages
// that we will use later to quickly navigate through it while going forward in time.
struct RosBagHandle {
  RosBagHandle() = delete;  // The rosbag API prevents anything else than initialization
  RosBagHandle(std::string const& bag_file, std::string const& topic) {
    bag_msgs.clear();
    bag.open(bag_file, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(topic);
    view = boost::shared_ptr<rosbag::View>(new rosbag::View(bag, rosbag::TopicQuery(topics)));
    for (rosbag::MessageInstance const m : *view) bag_msgs.push_back(m);
  }
  rosbag::Bag bag;
  boost::shared_ptr<rosbag::View> view;
  std::vector<rosbag::MessageInstance> bag_msgs;
};

}  // end namespace dense_map

#endif  // DENSE_MAP_ROS_UTILS_H_
