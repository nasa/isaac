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

#include <ff_common/init.h>
#include <ff_common/utils.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/highgui/highgui.hpp>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>

#include <boost/filesystem.hpp>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <TinyEXIF.h>

#include <dense_map_ros_utils.h>

#include <string>
#include <vector>
#include <fstream>

// Append sci cam images from a directory to a bag file.
// Their timestamp will be determined from the name of each image,
// which is expected to be the number of seconds since the epoch, such as
// mydir/1604944526.1389999.jpg. Append only images whose timestamp
// is within the range of times already in the bag.

// Any images on the sci cam topic already in the input bag (such as
// preview images published by the sci cam) will not be saved in the
// output bag.

DEFINE_string(input_bag, "", "The input bag.");

DEFINE_string(output_bag, "", "The output bag.");

DEFINE_string(sci_cam_dir, "", "The directory having the sci cam images.");

DEFINE_string(sci_cam_topic, "/hw/cam_sci/compressed", "The sci cam topic in the output bag.");

DEFINE_string(sci_cam_exif_topic, "/hw/sci_cam_exif", "The sci cam exif metadata topic the output bag.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if (FLAGS_input_bag == "" || FLAGS_output_bag == "" || FLAGS_sci_cam_dir == "" || FLAGS_sci_cam_topic == "" ||
      FLAGS_sci_cam_exif_topic == "") {
    std::cout << "Not all inputs were specified.\n";
    return 1;
  }

  // Make the directory where the output will go
  std::string out_dir = boost::filesystem::path(FLAGS_output_bag).parent_path().string();
  if (out_dir == "") out_dir = ".";
  if (!boost::filesystem::exists(out_dir)) {
    int status = mkdir(out_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status && errno != EEXIST) {
      std::cout << "Failed to create directory: " << out_dir << ".";
      return 1;
    }
  }

  // Open the output bag
  std::cout << "Opening the output bag file: " << FLAGS_output_bag << "\n";
  rosbag::Bag output_bag;
  output_bag.open(FLAGS_output_bag, rosbag::bagmode::Write);

  // Get the topics from the input bag
  std::vector<std::string> topics;
  dense_map::readTopicsInBag(FLAGS_input_bag, topics);

  // Open the input bag to write the input bag data. Skip any preexisting
  // sci cam topic in the input bag.
  rosbag::Bag input_bag;
  std::cout << "Opening the input bag file: " << FLAGS_input_bag << "\n";
  input_bag.open(FLAGS_input_bag, rosbag::bagmode::Read);

  double beg_time = -1.0, end_time = -1.0;
  rosbag::View view(input_bag, rosbag::TopicQuery(topics));
  for (rosbag::MessageInstance const m : view) {
    std::string topic = m.getTopic();
    if (topic == FLAGS_sci_cam_topic) continue;

    // If the current message is an image or a point cloud, get its
    // header timestamp. That is a more reliable time than
    // m.getTime(), which is the time the message got recorded, not
    // when it got made.
    double timestamp = -1.0;
    sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
    if (image_msg) {
      timestamp = image_msg->header.stamp.toSec();
    } else {
      sensor_msgs::CompressedImage::ConstPtr comp_image_msg =
        m.instantiate<sensor_msgs::CompressedImage>();
      if (comp_image_msg) {
        timestamp = comp_image_msg->header.stamp.toSec();
      } else {
        sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc_msg) {
          timestamp = pc_msg->header.stamp.toSec();
        }
      }
    }

    if (timestamp > 0.0) {
      // Record beg and end time from the headers
      if (beg_time < 0) beg_time = timestamp;
      end_time = timestamp;
    }

    output_bag.write(m.getTopic(), m.getTime(), m);
  }

  // Get the list of sci cam images
  std::vector<std::string> image_files;
  ff_common::ListFiles(FLAGS_sci_cam_dir, "jpg", &image_files);
  if (image_files.empty()) {
    std::cout << "No .jpg files found in " << FLAGS_sci_cam_dir << std::endl;
    return 1;
  }

  // Append the sci cam images to the output bag and also exif metadata
  for (int it = 0; it < image_files.size(); it++) {
    // Get the timestamp from the image name
    std::string timestamp_str = boost::filesystem::path(image_files[it]).stem().string();
    std::istringstream is(timestamp_str);
    double timestamp;
    if (!(is >> timestamp)) {
      std::cout << "Could not read the timestamp from image name: " << image_files[it] << std::endl;
      return 1;
    }

    if (timestamp < beg_time || timestamp > end_time) continue;

    ros::Time t(timestamp);

    // Parse the image EXIF metadata
    std::ifstream file(image_files[it].c_str(), std::ifstream::in | std::ifstream::binary);
    file.seekg(0, std::ios::end);
    std::streampos length = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<uint8_t> data(length);
    file.read(reinterpret_cast<char*>(data.data()), length);
    TinyEXIF::EXIFInfo imageEXIF(data.data(), length);

    // Save the metadata: timestamp, exposure, iso, aperture, and focal length
    std_msgs::Float64MultiArray exif;
    exif.layout.dim.resize(2);
    exif.layout.dim[0].label = "exif";
    exif.layout.dim[1].label = "";
    exif.layout.dim[0].size = dense_map::NUM_EXIF;
    exif.layout.dim[1].size = 1;
    exif.layout.dim[0].stride = dense_map::NUM_EXIF * 1;
    exif.layout.dim[1].stride = 1;
    exif.layout.data_offset = 0;

    exif.data.resize(dense_map::NUM_EXIF);
    exif.data[dense_map::TIMESTAMP] = timestamp;

    if (imageEXIF.Fields) {
#if 0
      std::cout
        << "Image Description  " << imageEXIF.ImageDescription << "\n"
        << "Image Resolution   " << imageEXIF.ImageWidth << "x" << imageEXIF.ImageHeight << " pix\n"
        << "Camera Model       " << imageEXIF.Make << " - " << imageEXIF.Model << "\n"
        << "Focal Length       " << imageEXIF.FocalLength << " mm" << std::endl
        << "Exposure (seconds) " << imageEXIF.ExposureTime << std::endl
        << "ISO (int)          " << imageEXIF.ISOSpeedRatings << std::endl
        << "aperture           " << imageEXIF.ApertureValue << std::endl;
#endif

      exif.data[dense_map::EXPOSURE_TIME] = imageEXIF.ExposureTime;
      exif.data[dense_map::ISO] = imageEXIF.ISOSpeedRatings;
      exif.data[dense_map::APERTURE] = imageEXIF.ApertureValue;
      exif.data[dense_map::FOCAL_LENGTH] = imageEXIF.FocalLength;

    } else {
      exif.data[dense_map::EXPOSURE_TIME] = -1.0;
      exif.data[dense_map::ISO] = -1.0;
      exif.data[dense_map::APERTURE] = -1.0;
      exif.data[dense_map::FOCAL_LENGTH] = -1.0;
    }

    output_bag.write(FLAGS_sci_cam_exif_topic, t, exif);

    // Form and save the compressed image message
    cv::Mat img;
    try {
      img = cv::imread(image_files[it], cv::IMREAD_COLOR);
    } catch (std::exception const& e) {
      // In case something is wrong with the jpg
      continue;
    }
    sensor_msgs::CompressedImage msg;
    msg.header.stamp = t;
    if (!cv::imencode(".jpg", img, msg.data)) {
      std::cout << "Could not compress image." << std::endl;
      return 1;
    }
    output_bag.write(FLAGS_sci_cam_topic, t, msg);
  }

  input_bag.close();
  output_bag.close();

  return 0;
}
