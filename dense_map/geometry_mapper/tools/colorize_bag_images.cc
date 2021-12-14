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

// A test tool to make the bag images in color, for testing purposes.
// The chosen colors are rather arbitrary. If the images are already
// in color, their colors will be altered as one can't tell if the
// image read in is color or grayscale, since it will have 3 channels
// either way. Compressed images will be decompressed and won't
// be compressed back.

#include <ff_common/init.h>
#include <ff_common/utils.h>
#include <dense_map_ros_utils.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <string>
#include <vector>
#include <fstream>

DEFINE_string(input_bag, "", "The input bag.");

DEFINE_string(output_bag, "", "The output bag.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if (FLAGS_input_bag == "" || FLAGS_output_bag == "") {
    std::cout << "Not all inputs were specified.\n";
    return 1;
  }

  int tmp_count = 10000;

  // Open the output bag
  std::cout << "Opening the output bag file: " << FLAGS_output_bag << "\n";
  rosbag::Bag output_bag;
  output_bag.open(FLAGS_output_bag, rosbag::bagmode::Write);

  // Get the topics from the input bag
  std::vector<std::string> topics;
  dense_map::readTopicsInBag(FLAGS_input_bag, topics);

  rosbag::Bag input_bag;
  std::cout << "Opening the input bag file: " << FLAGS_input_bag << "\n";
  input_bag.open(FLAGS_input_bag, rosbag::bagmode::Read);

  rosbag::View view(input_bag, rosbag::TopicQuery(topics));
  for (rosbag::MessageInstance const m : view) {
    bool have_image = false;
    cv::Mat image;
    std_msgs::Header header;

    // Try to read a compressed image
    sensor_msgs::CompressedImage::ConstPtr comp_image_msg
      = m.instantiate<sensor_msgs::CompressedImage>();
    if (comp_image_msg) {
      // Convert the compressed image data to cv::Mat
      try {
        image = cv::imdecode(cv::Mat(comp_image_msg->data), cv::IMREAD_COLOR);
      } catch (cv_bridge::Exception const& e) {
        LOG(ERROR) << "Unable to convert compressed image to bgr8.\n";
        continue;
      }

      // Use the header from the input image, including the timestamp
      header = comp_image_msg->header;
      have_image = true;
    }

    if (!have_image) {
      // Try to read a regular uncompressed image
      sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
      if (image_msg) {
        try {
          // Copy from the temporary pointer to a stable location
          (cv_bridge::toCvShare(image_msg, "bgr8")->image).copyTo(image);
        } catch (cv_bridge::Exception const& e) {
          LOG(ERROR) << "Unable to convert image to bgr8.\n";
          continue;
        }

        // Use the header from the input image, including the timestamp
        header = image_msg->header;
        have_image = true;
      }
    }

    if (!have_image) {
      // Write the message without any changes
      output_bag.write(m.getTopic(), m.getTime(), m);
      continue;
    }

    // Tweak the pixels to create color if the input is grayscale
    for (int row = 0; row < image.rows; row++) {
      for (int col = 0; col < image.cols; col++) {
        cv::Vec3b color = image.at<cv::Vec3b>(row, col);
        int delta = 60;
        color[0] = std::max(static_cast<int>(color[0]), delta) - delta;
        color[2] = std::max(static_cast<int>(color[2]), delta) - delta;
        image.at<cv::Vec3b>(row, col) = color;
      }
    }

    // Form the output image
    cv_bridge::CvImage cv_image;
    cv_image.image = image;
    cv_image.encoding = "bgr8";

//     std::ostringstream oss;
//     oss << "image_" << tmp_count << ".jpg";
//     std::string name = oss.str();
//     cv::imwrite(name, image);
//     tmp_count++;
//     std::cout << "--Writing: " << name << std::endl;

    sensor_msgs::Image out_image_msg;
    cv_image.toImageMsg(out_image_msg);

    // Use the header from the input image, including the timestamp
    out_image_msg.header = header;

    // Write it to the bag to the same topic
    output_bag.write(m.getTopic() + "_color", m.getTime(), out_image_msg);
  }

  input_bag.close();
  output_bag.close();

  return 0;
}
