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

#include <depth_from_mesh_utils.h>
#include <ff_common/init.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <glog/logging.h>

#include <string>

namespace dm = dense_map;
namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace lc = localization_common;
namespace mc = msg_conversions;

int main(int argc, char** argv) {
  std::string robot_config_file;
  std::string world;
  std::string sensor_frame;
  std::string output_file;
  po::options_description desc(
    "Adds depth data to desired points at provided timestamps.  Uses a provided mesh and sequence of groundtruth poses "
    "to obtain the depth data.");
  desc.add_options()("help,h", "produce help message")(
    "timestamps-file", po::value<std::string>()->required(),
    "File containing a set of timestamps to generate depth data for. Each timestamp should be on a newline.")(
    "sensor-rays-file", po::value<std::string>()->required(),
    "File containing a set of rays from the sensor frame to generate depth data for. Each point has an assumed x "
    "offset of 1 meter from the sensor, so only y z pairs are required.  Each point pair should be on a newline.")(
    "groundtruth-directory", po::value<std::string>()->required(),
    "Directory containing groundtruth poses with timestamps as filenames.")(
    "mesh", po::value<std::string>()->required(), "Mesh used to provide depth data.")(
    "config-path,c", po::value<std::string>()->required(), "Path to astrobee config directory.")(
    "robot-config-file,r", po::value<std::string>(&robot_config_file)->default_value("config/robots/bumble.config"),
    "Robot config file")("world,w", po::value<std::string>(&world)->default_value("iss"), "World name")(
    "output-file", po::value<std::string>(&output_file)->default_value("depths.csv"),
    "Output file containing timestamp, depth, x, y values on each line. Timestamps for which depth data was not "
    "successfully loaded are not included in the output.")
    // TODO(rsoussan): Add soundsee sensor trafo to astrobee configs!
    ("sensor-frame,s", po::value<std::string>(&sensor_frame)->default_value("soundsee"),
     "Sensor frame to generate depth data in.");

  po::positional_options_description p;
  p.add("timestamps-file", 1);
  p.add("sensor-rays-file", 1);
  p.add("groundtruth-directory", 1);
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
  const std::string sensor_rays_file = vm["sensor-rays-file"].as<std::string>();
  const std::string groundtruth_directory = vm["groundtruth-directory"].as<std::string>();
  const std::string mesh_file = vm["mesh"].as<std::string>();
  const std::string config_path = vm["config-path"].as<std::string>();

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);

  if (!fs::exists(timestamps_file)) {
    LOG(FATAL) << "Timestamps file " << timestamps_file << " not found.";
  }
  if (!fs::exists(sensor_rays_file)) {
    LOG(FATAL) << "Sensor rays file " << sensor_rays_file << " not found.";
  }
  if (!fs::exists(groundtruth_directory)) {
    LOG(FATAL) << "Groundtruth directory " << groundtruth_directory << " not found.";
  }
  if (!fs::exists(mesh_file)) {
    LOG(FATAL) << "Mesh " << mesh_file << " not found.";
  }

  lc::SetEnvironmentConfigs(config_path, world, robot_config_file);
  config_reader::ConfigReader config;
  config.AddFile("geometry.config");
  if (!config.ReadFiles()) {
    LOG(FATAL) << "Failed to read config files.";
  }

  const auto sensor_t_rays = dm::LoadSensorRays(sensor_rays_file);
  const auto query_timestamps = dm::LoadTimestamps(timestamps_file);
  const auto body_T_sensor = mc::LoadEigenTransform(config, sensor_frame + "_transform");
  std::vector<std::string> groundtruth_sensor_frames{"nav_cam", "sci_cam", "haz_cam"};
  // TODO(rsoussan): Add option to pass these as args?
  std::vector<double> timestamp_offsets{0, 0, 0};
  std::vector<Eigen::Isometry3d> body_T_groundtruth_sensor_vec;
  for (const auto& sensor_frame : groundtruth_sensor_frames) {
    body_T_groundtruth_sensor_vec.emplace_back(mc::LoadEigenTransform(config, sensor_frame + "_transform"));
  }
  const auto groundtruth_pose_interpolater = dm::MakePoseInterpolater(
    groundtruth_directory, body_T_sensor, groundtruth_sensor_frames, body_T_groundtruth_sensor_vec, timestamp_offsets);
  const auto mesh_tree = dm::LoadMeshTree(mesh_file);
  const auto depths = dm::GetDepthData(query_timestamps, sensor_t_rays, groundtruth_pose_interpolater, *mesh_tree);
  int depth_count = 0;
  for (const auto& depth : depths) {
    if (depth) ++depth_count;
  }
  LOG(INFO) << "Got " << depth_count << " of " << query_timestamps.size() << " depths successfully.";
  dm::SaveDepthData(query_timestamps, sensor_t_rays, depths, output_file);
}
