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

// Uncompress, scale and/or convert to grayscale sci cam images from a bag

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

DEFINE_string(image_type, "grayscale", "The output image type. Options are 'grayscale' or 'color'.");

DEFINE_double(scale, 0.25, "The scale factor to apply to the sci cam images.");

DEFINE_string(input_sci_cam_topic, "/hw/cam_sci/compressed", "The sci cam topic in the input bag.");

DEFINE_string(output_sci_cam_topic, "/hw/cam_sci2", "The sci cam topic in the output bag.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if (FLAGS_input_bag == "" || FLAGS_output_bag == "") {
    std::cout << "Not all inputs were specified.\n";
    return 1;
  }

  if (FLAGS_image_type != "color" && FLAGS_image_type != "grayscale") {
    std::cout << "Image type must be 'color' or 'grayscale'.\n";
    return 1;
  }

  if (FLAGS_scale <= 0.0 || FLAGS_scale > 1.0) {
    std::cout << "The scale must be positive and no more than 1.\n";
    return 1;
  }

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
    std::string topic = m.getTopic();

    if (topic == FLAGS_output_sci_cam_topic)
      LOG(FATAL) << "Not expecting in the bag the output topic: " << FLAGS_output_sci_cam_topic;

    if (topic != FLAGS_input_sci_cam_topic) {
      // Write the message without any changes
      output_bag.write(m.getTopic(), m.getTime(), m);
      continue;
    }

    // From here on deal with the sci cam images only

    // Read and decompress it
    sensor_msgs::CompressedImage::ConstPtr comp_image_msg = m.instantiate<sensor_msgs::CompressedImage>();
    if (!comp_image_msg) LOG(FATAL) << "Expecting a compressed sci cam image.";

    // Convert the compressed image data to cv::Mat
    cv::Mat image;
    try {
      image = cv::imdecode(cv::Mat(comp_image_msg->data), cv::IMREAD_COLOR);
    } catch (cv_bridge::Exception const& e) {
      LOG(ERROR) << "Unable to convert compressed image to bgr8.";
      continue;
    }

    // Use this pointer to the output image. Do things this way to
    // avoid copying unless we have to.
    cv::Mat* image_ptr = &image;

    // Maybe resize
    cv::Mat resized_image;
    if (FLAGS_scale < 1.0) {
      cv::resize(*image_ptr, resized_image, cv::Size(), FLAGS_scale, FLAGS_scale,
                 cv::INTER_AREA);
      image_ptr = &resized_image;
    }

    // Maybe convert to grayscale
    cv::Mat grayscale_image;
    if (FLAGS_image_type == "grayscale") {
      cv::cvtColor(*image_ptr, grayscale_image, cv::COLOR_BGR2GRAY);
      image_ptr = &grayscale_image;
    }

    // Form the output image
    cv_bridge::CvImage cv_image;
    cv_image.image = *image_ptr;

    if (FLAGS_image_type == "color") {
      cv_image.encoding = "bgr8";
    } else if (FLAGS_image_type == "grayscale") {
      cv_image.encoding = "mono8";
    }

    sensor_msgs::Image out_msg;
    cv_image.toImageMsg(out_msg);

    // Use the header from the input image, including the timestamp
    out_msg.header = comp_image_msg->header;

    // Write it to the bag
    output_bag.write(FLAGS_output_sci_cam_topic, m.getTime(), out_msg);
  }

  input_bag.close();
  output_bag.close();

  return 0;
}
