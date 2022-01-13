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
#include <dense_map_utils.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <ff_common/utils.h>
#include <ff_common/thread.h>
#include <config_reader/config_reader.h>
#include <msg_conversions/msg_conversions.h>

#include <boost/filesystem.hpp>

#include <string>
#include <map>
#include <iostream>
#include <fstream>

// Pick images from which to build a map so that they bracket given
// sci cam images. Then pick extra images if needed.

DEFINE_string(ros_bag, "",
              "A ROS bag with recorded nav_cam, haz_cam, and sci_cam data.");

DEFINE_string(output_nav_cam_dir, "",
              "The path to a file saving the nav cam image list.");

DEFINE_string(nav_cam_topic, "/mgt/img_sampler/nav_cam/image_record",
              "The nav cam topic in the bag file.");

DEFINE_string(haz_cam_points_topic, "/hw/depth_haz/points",
              "The depth point cloud topic in the bag file.");
DEFINE_string(haz_cam_intensity_topic, "/hw/depth_haz/extended/amplitude_int",
              "The depth camera intensity topic in the bag file.");
DEFINE_string(sci_cam_topic, "/hw/cam_sci/compressed", "The sci cam topic in the bag file.");

DEFINE_double(max_time_between_images, 1.0e+10,
              "Select additional nav cam images to make the timestamps between any two "
              "consecutive such images be no more than this.");

DEFINE_double(bracket_len, 0.6,
              "Lookup sci and haz cam images only between consecutive nav cam images "
              "whose distance in time is no more than this (in seconds). It is assumed "
              "the robot moves slowly and uniformly during this time.");

DEFINE_double(nav_cam_to_sci_cam_offset_override_value,
              std::numeric_limits<double>::quiet_NaN(),
              "Override the value of nav_cam_to_sci_cam_timestamp_offset from the robot config "
              "file with this value.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  if (FLAGS_ros_bag.empty()) LOG(FATAL) << "The bag file was not specified.";

  if (FLAGS_output_nav_cam_dir.empty()) LOG(FATAL) << "The output nav cam dir was not specified.";

  if (FLAGS_bracket_len <= 0.0) LOG(FATAL) << "Must have a positive bracket length.";

  if (FLAGS_max_time_between_images <= 0.0) LOG(FATAL) << "Must use a positive value for --max_time_between_images.\n";

  if (!boost::filesystem::exists(FLAGS_output_nav_cam_dir))
    if (!boost::filesystem::create_directories(FLAGS_output_nav_cam_dir) ||
        !boost::filesystem::is_directory(FLAGS_output_nav_cam_dir))
      LOG(FATAL) << "Failed to create directory: " << FLAGS_output_nav_cam_dir;

  // Set up handles for reading data at given time stamp without
  // searching through the whole bag each time.
  dense_map::RosBagHandle nav_cam_handle(FLAGS_ros_bag, FLAGS_nav_cam_topic);
  dense_map::RosBagHandle sci_cam_handle(FLAGS_ros_bag, FLAGS_sci_cam_topic);
  dense_map::RosBagHandle haz_cam_points_handle(FLAGS_ros_bag, FLAGS_haz_cam_points_topic);
  dense_map::RosBagHandle haz_cam_intensity_handle(FLAGS_ros_bag, FLAGS_haz_cam_intensity_topic);

  std::vector<std::string> cam_types = {"nav_cam", "haz_cam", "sci_cam"};
  std::vector<camera::CameraParameters> cam_params;
  std::vector<Eigen::Affine3d>          nav_to_cam_trans;
  std::vector<double>                   nav_to_cam_timestamp_offset;
  Eigen::Affine3d                       nav_cam_to_body_trans;
  Eigen::Affine3d                       haz_cam_depth_to_image_transform;
  dense_map::readConfigFile(  // Inputs
    cam_types, "nav_cam_transform", "haz_cam_depth_to_image_transform",
    // Outputs
    cam_params, nav_to_cam_trans, nav_to_cam_timestamp_offset, nav_cam_to_body_trans,
    haz_cam_depth_to_image_transform);

  if (!std::isnan(FLAGS_nav_cam_to_sci_cam_offset_override_value)) {
    for (size_t it = 0; it < cam_types.size(); it++) {
      if (cam_types[it] == "sci_cam") {
        double new_val = FLAGS_nav_cam_to_sci_cam_offset_override_value;
        std::cout << "Overriding the value " << nav_to_cam_timestamp_offset[it]
                  << " of nav_cam_to_sci_cam_timestamp_offset with: " << new_val << std::endl;
        nav_to_cam_timestamp_offset[it] = new_val;
      }
    }
  }

  // This will be used for output
  std::map<double, double> haz_depth_to_image_timestamps, haz_image_to_depth_timestamps;

  std::vector<double> all_nav_cam_timestamps, all_haz_cam_timestamps, all_sci_cam_timestamps;

  // Here we count on the order of cam_types above
  double nav_cam_to_haz_cam_timestamp_offset = nav_to_cam_timestamp_offset[1];
  double nav_cam_to_sci_cam_timestamp_offset = nav_to_cam_timestamp_offset[2];

  std::cout << "nav_cam_to_haz_cam_timestamp_offset = " << nav_cam_to_haz_cam_timestamp_offset
            << "\n";
  std::cout << "nav_cam_to_sci_cam_timestamp_offset = " << nav_cam_to_sci_cam_timestamp_offset
            << "\n";

  // TODO(oalexan1): Make the logic below into functions

  // Read the haz cam data we will need
  double haz_cam_start_time = -1.0;
  std::vector<rosbag::MessageInstance> const& haz_cam_intensity_msgs
    = haz_cam_intensity_handle.bag_msgs;
  for (size_t it = 0; it < haz_cam_intensity_msgs.size(); it++) {
    sensor_msgs::Image::ConstPtr image_msg
      = haz_cam_intensity_msgs[it].instantiate<sensor_msgs::Image>();
    if (image_msg) {
      double haz_cam_time = image_msg->header.stamp.toSec();
      all_haz_cam_timestamps.push_back(haz_cam_time);

      if (haz_cam_start_time < 0) haz_cam_start_time = haz_cam_time;
    }
  }

  std::cout.precision(17);

  if (all_haz_cam_timestamps.empty())
    std::cout << "Warning: No haz cam images are present." << std::endl;
  else
    std::cout << "Number of haz cam images " << all_haz_cam_timestamps.size() << std::endl;

  // Read the sci cam data we will need (images and clouds)
  std::vector<rosbag::MessageInstance> const& sci_cam_msgs = sci_cam_handle.bag_msgs;
  std::cout << "Number of sci cam images " << sci_cam_msgs.size() << std::endl;
  std::set<double> sci_cam_set;
  for (size_t it = 0; it < sci_cam_msgs.size(); it++) {
    sensor_msgs::Image::ConstPtr image_msg = sci_cam_msgs[it].instantiate<sensor_msgs::Image>();
    if (image_msg) {
      double sci_cam_time = image_msg->header.stamp.toSec();
      all_sci_cam_timestamps.push_back(sci_cam_time);
    }
    // Can be compressed too
    sensor_msgs::CompressedImage::ConstPtr comp_image_msg =
      sci_cam_msgs[it].instantiate<sensor_msgs::CompressedImage>();
    if (comp_image_msg) {
      double sci_cam_time = comp_image_msg->header.stamp.toSec();
      all_sci_cam_timestamps.push_back(sci_cam_time);
      sci_cam_set.insert(sci_cam_time);
    }
  }
  if (all_sci_cam_timestamps.empty()) LOG(FATAL) << "No sci cam timestamps.";

  // See if to insert extra timestamps so we get good overlap when building
  // a sparse map. We'll also pick haz cam images in between any such pair
  // of bracketing images even if there's no sci cam there.
  std::vector<double> extra_sci_cam_timestamps;
  for (size_t sci_it = 0; sci_it + 1 < all_sci_cam_timestamps.size(); sci_it++) {
    double beg = all_sci_cam_timestamps[sci_it];
    double end = all_sci_cam_timestamps[sci_it + 1];
    if (end - beg > FLAGS_max_time_between_images) {
      int num = ceil((end - beg) / FLAGS_max_time_between_images);
      for (int extra_it = 1; extra_it < num; extra_it++) {
        extra_sci_cam_timestamps.push_back(beg + extra_it * (end - beg) / num);
      }
    }
  }

  std::vector<double> sci_cam_timestamps_plus_extra = all_sci_cam_timestamps;
  for (size_t extra_it = 0; extra_it < extra_sci_cam_timestamps.size(); extra_it++) {
    sci_cam_timestamps_plus_extra.push_back(extra_sci_cam_timestamps[extra_it]);
  }
  std::sort(sci_cam_timestamps_plus_extra.begin(), sci_cam_timestamps_plus_extra.end());

  // Read the nav cam data we will need
  std::vector<rosbag::MessageInstance> const& nav_cam_msgs = nav_cam_handle.bag_msgs;
  std::cout << "Number of nav cam images " << nav_cam_msgs.size() << std::endl;
  for (size_t it = 0; it < nav_cam_msgs.size(); it++) {
    sensor_msgs::Image::ConstPtr image_msg = nav_cam_msgs[it].instantiate<sensor_msgs::Image>();
    if (image_msg) {
      double nav_cam_time = image_msg->header.stamp.toSec();
      all_nav_cam_timestamps.push_back(nav_cam_time);
    }
  }
  if (all_nav_cam_timestamps.empty()) LOG(FATAL) << "No nav cam timestamps. Check the nav cam topic.";

  std::vector<double> nav_cam_timestamps;

  // We make the bracket a tiny bit smaller as further down we will operate
  // with large numbers, in seconds since epoch, which is 10^9 of them,
  // and there's some concern because of floating point arithmetic
  // with doubles the bracket won't be respected fully.
  double d = FLAGS_bracket_len * 0.999;

  // Bracket each sci cam image at time s by two nav cam images at times t1 and t2.
  // Find the smallest t1 with t1 >= s - d/2 && t1 <= s.
  // these counters only increase and keep track of where we are in
  // time in the list of nav and sci messages.
  // Keep the nav image whose time plus FLAGS_bracket_len is <= the given sci cam image.
  int num_nav = all_nav_cam_timestamps.size();
  int num_sci_plus_extra = sci_cam_timestamps_plus_extra.size();
  int num_sci = sci_cam_set.size();
  int nav_cam_pos = 0;  // used to narrow down the lookup
  int num_bracketed_sci_cams = 0;
  for (int sci_it = 0; sci_it < num_sci_plus_extra; sci_it++) {
    for (int nav_it1 = nav_cam_pos; nav_it1 < num_nav; nav_it1++) {
      // Adjust for timestamp offsets
      double t1 = all_nav_cam_timestamps[nav_it1];
      double s = sci_cam_timestamps_plus_extra[sci_it] - nav_cam_to_sci_cam_timestamp_offset;

      bool is_good1 = (t1 >= s - d/2.0 && t1 <= s);

      if (is_good1) {
        // Got the left bracket.
        nav_cam_pos = nav_it1;  // save this for the future

        // Now get the right bracket
        // Use an int counter, as with unsigned int values subtracting is dangerous
        for (int nav_it2 = num_nav - 1; nav_it2 >= 0; nav_it2--) {
          double t2 = all_nav_cam_timestamps[nav_it2];

          bool is_good2 = (s < t2 && t2 <= s + d/2.0);
          if (is_good2) {
            nav_cam_timestamps.push_back(all_nav_cam_timestamps[nav_it1]);
            nav_cam_timestamps.push_back(all_nav_cam_timestamps[nav_it2]);
            std::cout << std::setprecision(17) << std::fixed << "For ";
            if (sci_cam_set.find(sci_cam_timestamps_plus_extra[sci_it]) != sci_cam_set.end())
              std::cout << "sci_cam ";
            else
              std::cout << "extra   ";
            std::cout << "timestamp "
                      << sci_cam_timestamps_plus_extra[sci_it] << std::setprecision(4)
                      << ", adj_stamp - nav1 and nav2 - adj_stamp: " << s - t1 << ' ' << t2 - s
                      << std::endl;

            if (sci_cam_set.find(sci_cam_timestamps_plus_extra[sci_it]) != sci_cam_set.end())
              num_bracketed_sci_cams++;

            break;  // Found what we needed, stop this loop
          }
        }

        break;  // Found what we needed, stop this loop
      }
    }
  }

  if (num_bracketed_sci_cams == num_sci)
    std::cout << "Bracketed all " << num_sci << " sci_cam timestamps." << std::endl;
  else
    std::cout << "Bracketed only " << num_bracketed_sci_cams << " sci cam timestamps "
              << "out of " << num_sci << ", not counting the extra timestamps. "
              << "Perhaps a bigger bracket may be desired but that may result in inaccurate "
              << "timestamp interpolation later. Ensure existing images are removed before new "
              << "ones are added.\n";

  // Ensure the timestamps are sorted
  std::sort(nav_cam_timestamps.begin(), nav_cam_timestamps.end());

  // Write the images to disk
  std::cout << "Writing the images to: " << FLAGS_output_nav_cam_dir << std::endl;
  int bag_pos = 0;
  bool save_grayscale = true;  // For feature matching need grayscale
  for (size_t nav_it = 0; nav_it < nav_cam_timestamps.size(); nav_it++) {
    double found_time = -1.0;
    cv::Mat image;
    if (!dense_map::lookupImage(nav_cam_timestamps[nav_it], nav_cam_handle.bag_msgs, save_grayscale, image,
                                bag_pos,  // will change each time
                                found_time))
      LOG(FATAL) << "Could not find image at the desired time.";

    char filename_buffer[1024];
    snprintf(filename_buffer, sizeof(filename_buffer), "%s/%10.7f.jpg",
             FLAGS_output_nav_cam_dir.c_str(),
             nav_cam_timestamps[nav_it]);
    std::cout << "Writing: " << filename_buffer << std::endl;
    cv::imwrite(filename_buffer, image);
  }
  return 0;
}
