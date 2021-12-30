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

#include <dense_map_ros_utils.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>

#include <string>
#include <vector>
#include <fstream>

// Extract clouds a ROS bag.

DEFINE_string(topic, "/hw/depth_haz/points", "The topic having the clouds.");
DEFINE_string(output_directory, "", "Directory for writing the output imagery.");
DEFINE_string(output_format, "%06i", "Format string for writing the output data.");
DEFINE_double(start, 0, "Start extracting this many seconds into the bag.");
DEFINE_double(duration, 1e+100, "Extract this many seconds from the bag. Default: extract the full bag.");
DEFINE_bool(use_timestamp_as_image_name, false,
            "Let the acquisition timestamp (in seconds since Epoch) be the output image name.");

void form_filename(int seq, double timestamp, char* filename_buffer, int buffer_len) {
  if (!FLAGS_use_timestamp_as_image_name)
    snprintf(filename_buffer, buffer_len, FLAGS_output_format.c_str(), seq);
  else
    snprintf(filename_buffer, buffer_len, "%10.7f", timestamp);
}

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " bag.bag\n";
    exit(0);
  }

  char filename_buffer[1024];

  rosbag::Bag bag;
  std::cout << "Opening bag file " << argv[1] << ".\n";
  bag.open(argv[1], rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(FLAGS_topic);
  std::cout << "Looking for topic: " << FLAGS_topic << "\n";

  std::string output_directory = FLAGS_output_directory;
  if (output_directory.empty()) {
    char* temp = strdup(argv[1]);
    std::string base = std::string(basename(temp));
    output_directory = base.substr(0, base.length() - 4) + "_clouds";
  }

  // Create the output directory, if missing
  int status = mkdir(output_directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (status && errno != EEXIST) {
    LOG(ERROR) << "Failed to create directory " << output_directory << ".";
    exit(1);
  }

  double beg_time = -1, curr_time = -1;
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::cout << "Copying at most " << view.size() << " frames from the bag file.\n";
  for (rosbag::MessageInstance const m : view) {
    sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (pc_msg) {
      ros::Time stamp = pc_msg->header.stamp;
      curr_time = stamp.toSec();

      // Set up the output filename
      form_filename(pc_msg->header.seq, curr_time, filename_buffer, sizeof(filename_buffer));
      std::string name(filename_buffer);
      name = output_directory + "/" + name + ".ply";

      if (beg_time < 0) beg_time = curr_time;
      if (curr_time - beg_time < FLAGS_start ||
          curr_time - beg_time > FLAGS_start + FLAGS_duration) {
        continue;
      }

      pcl::PointCloud<pcl::PointXYZ> pc;
      dense_map::msgToPcl(pc_msg, pc);
      if (static_cast<int>(pc.points.size()) != static_cast<int>(pc_msg->width * pc_msg->height))
        LOG(FATAL) << "Extracted point cloud size does not agree with original size.";

      std::cout << "size is " << pc_msg->width << ' ' << pc_msg->height << std::endl;

      std::cout << "Writing: " << name << "\n";
      pcl::io::savePLYFileASCII(name, pc);
    }
  }

  bag.close();
}
