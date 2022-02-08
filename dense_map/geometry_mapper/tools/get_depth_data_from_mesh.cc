/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
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

#include <dense_map/depth_data_generator.h>
#include <ff_common/init.h>
// TODO(rsoussan): Move set configs fcn to ff_common??
#include <localization_common/utilities.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <glog/logging.h>

namespace po = boost::program_options;
namespace lc = localization_common;

int main(int argc, char** argv) {
  std::string robot_config_file;
  std::string world;
  po::options_description desc(
    "Adds depth data to desired points at provided timestamps.  Uses a provided mesh and sequence of groundtruth poses "
    "to obtain the depth data.");
  desc.add_options()("help,h", "produce help message")
("timestamps-file", po::value<std::string>()->required(),
    "File containing a set of timestamps to generate depth data for. Each timestamped should be on a newline.")
("image-points-file", po::value<std::string>()->required(),
    "File containing a set of image points to generate depth data for. Each image point should be on a newline.")
("sensor-frame", po::value<std::string>()->required(),
    "Sensor frame to generate depth data in.")
(
    "gt-poses-file", po::value<std::string>()->required(),
    "File containing sequence of groundtruth poses.")
("gt-pose-frame", po::value<std::string>()->required(),
    "Frame that the groundtruth poses are in.")
("mesh", po::value<std::string>()->required(),
    "Mesh used to provide depth data.")
    "config-path,c", po::value<std::string>()->required(), "Path to astrobee config directory.")(
    "robot-config-file,r", po::value<std::string>(&robot_config_file)->default_value("config/robots/bumble.config"),
    "Robot config file")("world,w", po::value<std::string>(&world)->default_value("iss"), "World name");
  po::positional_options_description p;
  p.add("timestamps-file", 1);
  p.add("image-points-file", 1);
  p.add("sensor-frame", 1);
  p.add("gt-poses-file", 1);
  p.add("gt-pose-frame", 1);
  p.add("mesh", 1);
  p.add("config-path", 1);
  po::variables_map vm;
  try {
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    if (vm.count("help") || (argc <= 1)) {
      std::cout << desc << "\n";
      return 1;
    }
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  const std::string timestamps_file = vm["timestamps-file"].as<std::string>();
  const std::string image_points_file = vm["image-points-file"].as<std::string>();
  const std::string sensor_frame = vm["sensor-frame"].as<std::string>();
  const std::string gt_poses_file = vm["gt-poses-file"].as<std::string>();
  const std::string gt_pose_frame = vm["gt-pose-frame"].as<std::string>();
  const std::string mesh = vm["mesh"].as<std::string>();
  const std::string config_path = vm["config-path"].as<std::string>();

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);

  if (!boost::filesystem::exists(timestamps_file)) {
    LOG(FATAL)("Timestamps file " << timestamps_file << " not found.");
  }
  if (!boost::filesystem::exists(image_points_file)) {
    LOG(FATAL)("Image points file " << image_points_file << " not found.");
  }
  if (!boost::filesystem::exists(gt_poses_file)) {
    LOG(FATAL)("Groundtruth poses file " << gt_poses_file << " not found.");
  }
  if (!boost::filesystem::exists(mesh_file)) {
    LOG(FATAL)("Mesh " << mesh_file << " not found.");
  }

  lc::SetEnvironmentConfigs(config_path, world, robot_config_file);
  config_reader::ConfigReader config;
  config.AddFile("geometry.config");
  if (!config.ReadFiles()) {
    LOG(FATAL)("Failed to read config files.");
  }

  // TODO(rsoussan): get vec of timestamps, make struct for image points?, get vec of gt poses, get mesh, get
  // gt_sensor_T_sensor! Load timestamps like in calibration package! load from csv! how is this done there? same for
  // image poiints! Use oleg's tools to load gt poses, mesh, and sensor frame Is soundsee sensor frame provided
  // somewhere? add option to pass in command line using a file?

  dense_map::DepthDataGenerator depth_data_generator(timestamps, image_points, gt_poses, mesh, gt_sensor_T_sensor);
  depth_data_generator.GetDepthData();
}
