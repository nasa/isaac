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

#include <dense_map_utils.h>
#include <ff_common/init.h>
#include <localization_common/pose_interpolater.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>
#include <texture_processing.h>

#include <acc/bvh_tree.h>
#include <mve/mesh.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <glog/logging.h>

namespace dm = dense_map;
namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace lc = localization_common;
namespace mc = msg_conversions;

boost::optional<double> Depth(const Eigen::Vector3d& sensor_t_ray, const Eigen::Isometry3d& world_T_sensor,
                              const BVHTree& bvh_tree, double min_depth = 1e-3, double max_depth = 1e2) {
  const Eigen::Vector3d world_F_sensor_t_ray = world_T_sensor.linear() * sensor_t_ray;
  // Create ray centered at world_t_sensor
  BVHTree::Ray bvh_ray;
  bvh_ray.origin = dm::eigen_to_vec3f(world_T_sensor.translation());
  bvh_ray.dir = dm::eigen_to_vec3f(world_F_sensor_t_ray);
  bvh_ray.dir.normalize();
  bvh_ray.tmin = min_depth;
  bvh_ray.tmax = max_depth;

  // Get intersection if it exists
  BVHTree::Hit hit;
  if (bvh_tree.intersect(bvh_ray, &hit)) return hit.t;
  return boost::none;
}

lc::Time LoadTimestamp(const std::string& filename, const double timestamp_offset = 0.0) {
  const std::string timestamp_string = filename.substr(0, filename.find("_"));
  if (timestamp_string.empty()) LOG(FATAL) << "Failed to load timestamp from filename.";
  return std::stod(timestamp_string) + timestamp_offset;
}

void LoadTimestampsAndPoses(const std::string& directory, const std::string& sensor_name,
                            std::vector<lc::Time>& timestamps, std::vector<Eigen::Isometry3d>& poses,
                            const Eigen::Isometry3d& poses_sensor_T_sensor = Eigen::Isometry3d::Identity(),
                            const double timestamp_offset = 0.0) {
  for (const auto& file : fs::recursive_directory_iterator(directory)) {
    const std::string filename = file.path().filename().string();
    if (boost::algorithm::ends_with(filename, "world.txt") && filename.find(sensor_name) != std::string::npos) {
      timestamps.emplace_back(LoadTimestamp(filename, timestamp_offset));
      Eigen::Affine3d affine_pose;
      dm::readAffine(affine_pose, filename);
      const Eigen::Isometry3d world_T_poses_sensor(affine_pose.matrix());
      poses.push_back(world_T_poses_sensor * poses_sensor_T_sensor);
    }
  }
}

// Assumes each timestamp is on a newline of the file
std::vector<lc::Time> LoadTimestamps(const std::string& timestamps_filename) {
  std::vector<lc::Time> timestamps;
  std::ifstream timestamps_file(timestamps_filename);
  std::string file_line;
  lc::Time timestamp;
  while (std::getline(timestamps_file, file_line)) {
    std::istringstream line_ss(file_line);
    line_ss >> timestamp;
    timestamps.emplace_back(timestamp);
  }
  return timestamps;
}

lc::PoseInterpolater MakePoseInterpolater(
  const std::string& directory, const std::string& sensor_name,
  const Eigen::Isometry3d& poses_sensor_T_sensor = Eigen::Isometry3d::Identity(), const double timestamp_offset = 0.0) {
  std::vector<lc::Time> timestamps;
  std::vector<Eigen::Isometry3d> poses;
  LoadTimestampsAndPoses(directory, sensor_name, timestamps, poses, poses_sensor_T_sensor, timestamp_offset);
  return lc::PoseInterpolater(timestamps, poses);
}

std::vector<Eigen::Vector3d> LoadSensorRays(const std::string& sensor_rays_filename) {
  std::vector<Eigen::Vector3d> sensor_t_rays;
  std::ifstream sensor_rays_file(sensor_rays_filename);
  std::string file_line;
  double y, z;
  while (std::getline(sensor_rays_file, file_line)) {
    std::istringstream line_ss(file_line);
    // TODO(rsoussan): Correct order?
    line_ss >> z;
    line_ss >> y;
    // Assumes each point is sampled from a y, z grid with an x offset of 1.0
    const Eigen::Vector3d sensor_t_ray(1.0, y, z);
    sensor_t_rays.emplace_back(sensor_t_ray);
  }
  return sensor_t_rays;
}

std::shared_ptr<BVHTree> LoadMeshTree(const std::string& mesh_file) {
  mve::TriangleMesh::Ptr mesh;
  std::shared_ptr<mve::MeshInfo> mesh_info;
  std::shared_ptr<tex::Graph> graph;
  std::shared_ptr<BVHTree> bvh_tree;
  dm::loadMeshBuildTree(mesh_file, mesh, mesh_info, graph, bvh_tree);
  return bvh_tree;
}

std::vector<boost::optional<double>> GetDepthData(const std::vector<lc::Time>& timestamps,
                                                  const std::vector<Eigen::Vector3d>& sensor_t_rays,
                                                  const lc::PoseInterpolater& groundtruth_pose_interpolater,
                                                  const BVHTree& bvh_tree) {
  std::vector<boost::optional<double>> depths;
  for (const auto& timestamp : timestamps) {
    const auto world_T_sensor = groundtruth_pose_interpolater.Interpolate(timestamp);
    if (!world_T_sensor) {
      LOG(ERROR) << "Failed to get groundtruth pose at timstamp " << timestamp;
      depths.emplace_back(boost::none);
      continue;
    }
    for (const auto& sensor_t_ray : sensor_t_rays) {
      depths.emplace_back(Depth(sensor_t_ray, *world_T_sensor, bvh_tree));
    }
  }
  return depths;
}

void SaveDepthData(const std::vector<lc::Time>& timestamps, const std::vector<Eigen::Vector3d>& sensor_t_rays,
                   const std::vector<boost::optional<double>>& depths, const std::string& output_filename) {
  std::ofstream output_file;
  output_file.open(output_filename);
  for (int i = 0; i < timestamps.size(); ++i) {
    if (!depths[i]) continue;
    output_file << timestamps[i] << " " << *(depths[i]) << " " << sensor_t_rays[i].z() << " " << sensor_t_rays[i].x()
                << std::endl;
  }
  output_file.close();
}

int main(int argc, char** argv) {
  std::string robot_config_file;
  std::string world;
  std::string sensor_frame;
  std::string groundtruth_sensor_frame;
  std::string output_file;
  double timestamp_offset;
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
     "Sensor frame to generate depth data in.")(
      "groundtruth-sensor-frame,g", po::value<std::string>(&groundtruth_sensor_frame)->default_value("nav_cam"),
      "Sensor frame of groundtruth poses.")("timestamp-offset,o",
                                            po::value<double>(&timestamp_offset)->default_value(0.0),
                                            "Timestamp offset for the sensor used to generate groundtruth poses.");

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

  const auto sensor_t_rays = LoadSensorRays(sensor_rays_file);
  const auto query_timestamps = LoadTimestamps("");
  const auto body_T_sensor = mc::LoadEigenTransform(config, sensor_frame + "_transform");
  const auto body_T_groundtruth_sensor = mc::LoadEigenTransform(config, groundtruth_sensor_frame + "_transform");
  const Eigen::Isometry3d groundtruth_sensor_T_sensor = body_T_groundtruth_sensor.inverse() * body_T_sensor;
  const auto groundtruth_pose_interpolater = MakePoseInterpolater(groundtruth_directory, groundtruth_sensor_frame,
                                                                  groundtruth_sensor_T_sensor, timestamp_offset);
  const auto mesh_tree = LoadMeshTree(mesh_file);
  const auto depths = GetDepthData(query_timestamps, sensor_t_rays, groundtruth_pose_interpolater, *mesh_tree);
  SaveDepthData(query_timestamps, sensor_t_rays, depths, output_file);
}
