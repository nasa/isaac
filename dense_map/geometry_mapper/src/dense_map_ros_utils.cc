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

#include <dense_map_ros_utils.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen_conversions/eigen_msg.h>
#include <ff_msgs/VisualLandmarks.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Float64MultiArray.h>

#include <glog/logging.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>

namespace dense_map {

// Publish a given pose
void PublishTF(const Eigen::Affine3d& publish_tf, std::string const& parent_frame, std::string const& child_frame,
               ros::Time const& timestamp, tf2_ros::TransformBroadcaster& tf_publisher) {
  Eigen::Quaterniond rtoq(publish_tf.linear());
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = timestamp;
  transformStamped.header.frame_id = parent_frame;
  transformStamped.child_frame_id = child_frame;
  transformStamped.transform.translation.x = static_cast<float>(publish_tf.translation()(0));
  transformStamped.transform.translation.y = static_cast<float>(publish_tf.translation()(1));
  transformStamped.transform.translation.z = static_cast<float>(publish_tf.translation()(2));
  transformStamped.transform.rotation.x = static_cast<float>(rtoq.x());
  transformStamped.transform.rotation.y = static_cast<float>(rtoq.y());
  transformStamped.transform.rotation.z = static_cast<float>(rtoq.z());
  transformStamped.transform.rotation.w = static_cast<float>(rtoq.w());
  tf_publisher.sendTransform(transformStamped);
}

// Parse geometry_msgs::PoseStamped or ff_msgs::VisualLandmarks
// poses from a bag for given topic.
void readBagPoses(std::string const& bag_file, std::string const& topic, StampedPoseStorage& poses) {
  if (!boost::filesystem::exists(bag_file)) LOG(FATAL) << "Bag does not exist: " << bag_file;

  poses.clear();
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for (rosbag::MessageInstance const m : view) {
    geometry_msgs::PoseStamped::ConstPtr pose_msg = m.instantiate<geometry_msgs::PoseStamped>();
    if (pose_msg) {
      ros::Time stamp = pose_msg->header.stamp;
      double curr_time = stamp.toSec();
      Eigen::Affine3d cam_pose;
      tf::poseMsgToEigen(pose_msg->pose, cam_pose);
      poses.addPose(cam_pose, stamp.toSec());
    } else {
      ff_msgs::VisualLandmarks::ConstPtr landmark_msg = m.instantiate<ff_msgs::VisualLandmarks>();
      if (landmark_msg) {
        ros::Time stamp = landmark_msg->header.stamp;
        double curr_time = stamp.toSec();
        Eigen::Affine3d cam_pose;
        tf::poseMsgToEigen(landmark_msg->pose, cam_pose);
        poses.addPose(cam_pose, stamp.toSec());
      }
    }
  }
}

void readBagImageTimestamps(std::string const& bag_file, std::string const& topic, std::vector<double>& timestamps) {
  if (!boost::filesystem::exists(bag_file)) LOG(FATAL) << "Bag does not exist: " << bag_file;

  timestamps.clear();
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(topic);

  double prev_time = -1.0;
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for (rosbag::MessageInstance const m : view) {
    sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
    if (image_msg) {
      double curr_time = image_msg->header.stamp.toSec();
      if (prev_time >= curr_time) {
        LOG(ERROR) << "In bag file: " << bag_file << ", earlier timestamp " << prev_time
                   << " is no less than current timestamp " << curr_time;
      }
      timestamps.push_back(curr_time);
      prev_time = curr_time;
    }
  }
}

// Given a bag view, for each topic in the view read the vector of
// messages for that topic, sorted by message header timestamp. Only
// the following sensor types are supported: //sensor_msgs::Image,
// sensor_msgs::CompressedImage, and sensor_msgs::PointCloud2.
void indexMessages(rosbag::View& view,  // view can't be made const
                   std::map<std::string, std::vector<rosbag::MessageInstance>>& bag_map) {
  bag_map.clear();

  // First put the data in maps so that we can sort it
  std::map<std::string, std::map<double, rosbag::MessageInstance>> local_map;
  for (rosbag::MessageInstance const m : view) {
    // Check for regular image message
    sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
    if (image_msg) {
      double  curr_time = image_msg->header.stamp.toSec();
      local_map[m.getTopic()].insert(std::make_pair(curr_time, m));
      continue;
    }

    // Check for compressed image message
    sensor_msgs::CompressedImage::ConstPtr comp_image_msg =
      m.instantiate<sensor_msgs::CompressedImage>();
    if (comp_image_msg) {
      double  curr_time = comp_image_msg->header.stamp.toSec();
      local_map[m.getTopic()].insert(std::make_pair(curr_time, m));
      continue;
    }

    // Check for cloud
    sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (pc_msg) {
      double curr_time = pc_msg->header.stamp.toSec();
      local_map[m.getTopic()].insert(std::make_pair(curr_time, m));
      continue;
    }
  }

  // Add the data in sorted order
  for (auto topic_it = local_map.begin(); topic_it != local_map.end() ; topic_it++) {
    auto& topic_name = topic_it->first;  // alias
    auto& topic_map = topic_it->second;  // alias

    for (auto msg_it = topic_map.begin(); msg_it != topic_map.end() ; msg_it++)
      bag_map[topic_name].push_back(msg_it->second);
  }
}

// Read exif data from a bag
void readExifFromBag(std::vector<rosbag::MessageInstance> const& bag_msgs,
                     std::map<double, std::vector<double> >& exif) {
  exif.clear();

  for (size_t exif_it = 0; exif_it < bag_msgs.size(); exif_it++) {
    // Check for cloud
    std_msgs::Float64MultiArray::ConstPtr exif_msg = bag_msgs[exif_it].instantiate<std_msgs::Float64MultiArray>();
    if (!exif_msg) continue;

    std::vector<double> exif_vec;
    for (int it = 0; it < dense_map::NUM_EXIF; it++) {
      exif_vec.push_back(exif_msg->data[it]);
    }

    double timestamp = exif_msg->data[dense_map::TIMESTAMP];
    exif[timestamp] = exif_vec;
  }
}

// Find an image at the given timestamp or right after it. We assume
// that during repeated calls to this function we always travel
// forward in time, and we keep track of where we are in the bag using
// the variable bag_pos that we update as we go.
bool lookupImage(double desired_time, std::vector<rosbag::MessageInstance> const& bag_msgs,
                   bool save_grayscale, cv::Mat& image, int& bag_pos, double& found_time) {
  found_time = -1.0;  // Record the time at which the image was found
  int num_msgs = bag_msgs.size();
  double prev_image_time = -1.0;

  for (int local_pos = bag_pos; local_pos < num_msgs; local_pos++) {
    bag_pos = local_pos;  // save this for exporting

    // Check for uncompressed images
    sensor_msgs::Image::ConstPtr image_msg
      = bag_msgs[local_pos].instantiate<sensor_msgs::Image>();
    if (image_msg) {
      ros::Time stamp = image_msg->header.stamp;
      found_time = stamp.toSec();

      // Sanity check: We must always travel forward in time
      if (found_time < prev_image_time) {
        LOG(ERROR) << "Found images in a bag not in chronological order. Caution advised.\n"
                   << std::fixed << std::setprecision(17)
                   << "Times in wrong order: " << prev_image_time << ' ' << found_time << ".\n";
        continue;
      }
      prev_image_time = found_time;

      if (found_time >= desired_time) {
        try {
          if (!save_grayscale) {
            // Do a copy, as image_msg may soon run out of scope
            (cv_bridge::toCvShare(image_msg, "bgr8")->image).copyTo(image);
          } else {
            // For some reason, looking up the image as grayscale does not work,
            // so first it needs to be looked up as color and then converted.
            cv::Mat tmp_image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
            cv::cvtColor(tmp_image, image, cv::COLOR_BGR2GRAY);
          }
        } catch (cv_bridge::Exception const& e) {
          ROS_ERROR_STREAM("Unable to convert " << image_msg->encoding.c_str()
                           << " image to bgr8.");
          return false;
        }
        return true;
      }
    }

    // Check for compressed images
    sensor_msgs::CompressedImage::ConstPtr comp_image_msg =
      bag_msgs[local_pos].instantiate<sensor_msgs::CompressedImage>();
    if (comp_image_msg) {
      ros::Time stamp = comp_image_msg->header.stamp;
      found_time = stamp.toSec();

      // Sanity check: We must always travel forward in time
      if (found_time < prev_image_time) {
        LOG(ERROR) << "Found images in a bag not in chronological order. Caution advised.\n"
                   << std::fixed << std::setprecision(17)
                   << "Times in wrong order: " << prev_image_time << ' ' << found_time << ".\n";
        continue;
      }
      prev_image_time = found_time;

      if (found_time >= desired_time) {
        try {
          if (!save_grayscale) {
            image = cv::imdecode(cv::Mat(comp_image_msg->data), cv::IMREAD_COLOR);
          } else {
            cv::Mat tmp_image = cv::imdecode(cv::Mat(comp_image_msg->data), cv::IMREAD_COLOR);
            cv::cvtColor(tmp_image, image, cv::COLOR_BGR2GRAY);
          }
        } catch (cv_bridge::Exception const& e) {
          ROS_ERROR_STREAM("Unable to convert compressed image to bgr8.");
          return false;
        }
        return true;
      }
    }
  }
  return false;
}

// Find the closest depth cloud to given timestamp (before or after
// it). Return an empty one if one cannot be found closer in time than
// max_time_diff. Store it as a cv::Mat of vec3f values. We assume
// that during repeated calls to this function we always travel
// forward in time, and we keep track of where we are in the bag using
// the variable bag_pos that we update as we go.
bool lookupCloud(double desired_time, std::vector<rosbag::MessageInstance> const& bag_msgs,
                   double max_time_diff, cv::Mat& cloud, int& bag_pos, double& found_time) {
  int num_msgs = bag_msgs.size();
  double prev_time = -1.0;
  found_time = -1.0;
  sensor_msgs::PointCloud2::ConstPtr prev_pc_msg = NULL;

  // Initialize the output as an empty matrix of Vec3f
  cloud.create(0, 0, CV_32FC3);

  for (int local_pos = bag_pos; local_pos < num_msgs; local_pos++) {
    // Check for cloud
    sensor_msgs::PointCloud2::ConstPtr curr_pc_msg
      = bag_msgs[local_pos].instantiate<sensor_msgs::PointCloud2>();
    if (!curr_pc_msg) continue;
    double curr_time = curr_pc_msg->header.stamp.toSec();

    // Sanity check: We must always travel forward in time
    if (curr_time < prev_time) {
      LOG(ERROR) << "Found images in a bag not in chronological order. Caution advised.\n"
                 << std::fixed << std::setprecision(17)
                 << "Times in wrong order: " << prev_time << ' ' << curr_time << ".\n";
      continue;
    }

    // We are not yet at the stage where we can make decisions, so just keep on going
    if (curr_time <= desired_time) {
      prev_time = curr_time;
      prev_pc_msg = curr_pc_msg;
      bag_pos = local_pos;
      continue;
    }

    // Now we are past desired_time. Need to see which timestamp is closer.
    if (std::abs(prev_time - desired_time) < std::abs(curr_time - desired_time)) {
      // Step back in time as the previous time is closer
      curr_time = prev_time;
      curr_pc_msg = prev_pc_msg;
    }

    double diff_time = std::abs(curr_time - desired_time);
    if (diff_time > max_time_diff) {
      std::cout << "Found a gap of " << diff_time << " seconds between haz cam intensity and "
                << "depth timestamps. This is too big. Skipping this timestamp.\n";
      return false;
    }

    found_time = curr_time;

    // Decode the cloud
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromROSMsg(*curr_pc_msg, pc);
    if (static_cast<int>(pc.points.size()) !=
        static_cast<int>(curr_pc_msg->width * curr_pc_msg->height))
      LOG(FATAL) << "Extracted point cloud size does not agree with original size.";

    cloud = cv::Mat::zeros(curr_pc_msg->height, curr_pc_msg->width, CV_32FC3);
    for (int row = 0; row < curr_pc_msg->height; row++) {
      for (int col = 0; col < curr_pc_msg->width; col++) {
        int count = row * curr_pc_msg->width + col;
        cloud.at<cv::Vec3f>(row, col)
          = cv::Vec3f(pc.points[count].x, pc.points[count].y, pc.points[count].z);
      }
    }

    // Found the cloud
    return true;
  }

  return false;
}

// Read the list of topics in a bag while avoiding repetitions
void readTopicsInBag(std::string const& bag_file, std::vector<std::string>& topics) {
  if (!boost::filesystem::exists(bag_file)) LOG(FATAL) << "Bag does not exist: " << bag_file;

  topics.clear();

  rosbag::Bag bag(bag_file.c_str());
  rosbag::View view(bag);
  std::vector<const rosbag::ConnectionInfo*> connection_infos = view.getConnections();

  // Read first in a set to deal with any repetitions
  std::set<std::string> topics_set;
  BOOST_FOREACH(const rosbag::ConnectionInfo* info, connection_infos) {
    topics_set.insert(info->topic);
  }

  // Copy the unique names to a vector
  for (auto it = topics_set.begin(); it != topics_set.end(); it++) {
    topics.push_back(*it);
  }
}

}  // end namespace dense_map
