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
#include <depth_from_mesh_utils.h>
#include <localization_common/utilities.h>
#include <texture_processing.h>

#include <mve/mesh.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>

#include <glog/logging.h>

namespace dm = dense_map;
namespace fs = boost::filesystem;
namespace lc = localization_common;

namespace dense_map {

boost::optional<double> Depth(const Eigen::Vector3d& sensor_t_ray, const Eigen::Isometry3d& world_T_sensor,
                              const BVHTree& bvh_tree, double min_depth, double max_depth) {
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
  if (bvh_tree.intersect(bvh_ray, &hit))
    return hit.t;
  else
    VLOG(2) << "Failed to get mesh intersection.";
  return boost::none;
}

lc::Time LoadTimestamp(const std::string& filename, const double timestamp_offset) {
  const std::string timestamp_string = filename.substr(0, filename.find("_"));
  if (timestamp_string.empty()) LOG(FATAL) << "Failed to load timestamp from filename.";
  return std::stod(timestamp_string) + timestamp_offset;
}

void LoadTimestampsAndPoses(const std::string& directory, const std::string& sensor_name,
                            std::vector<lc::Time>& timestamps, std::vector<Eigen::Isometry3d>& poses,
                            const Eigen::Isometry3d& poses_sensor_T_sensor, const double timestamp_offset) {
  for (const auto& file : fs::recursive_directory_iterator(directory)) {
    const std::string filename = file.path().filename().string();
    if (boost::algorithm::ends_with(filename, "world.txt") && filename.find(sensor_name) != std::string::npos) {
      timestamps.emplace_back(LoadTimestamp(filename, timestamp_offset));
      Eigen::Affine3d affine_pose;
      if (!dm::readAffine(affine_pose, file.path().string())) {
        LOG(FATAL) << "Failed to read pose for filename " << filename;
      }
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

lc::PoseInterpolater MakePoseInterpolater(const std::string& directory, const Eigen::Isometry3d& body_T_sensor,
                                          const std::vector<std::string>& groundtruth_sensor_names,
                                          const std::vector<Eigen::Isometry3d>& body_T_groundtruth_sensor_vec,
                                          const std::vector<double>& timestamp_offsets) {
  std::vector<lc::Time> timestamps;
  std::vector<Eigen::Isometry3d> poses;
  for (int i = 0; i < groundtruth_sensor_names.size(); ++i) {
    LoadTimestampsAndPoses(directory, groundtruth_sensor_names[i], timestamps, poses,
                           (body_T_groundtruth_sensor_vec[i]).inverse() * body_T_sensor, timestamp_offsets[i]);
  }
  return lc::PoseInterpolater(timestamps, poses);
}

std::vector<Eigen::Vector3d> LoadSensorRays(const std::string& sensor_rays_filename, int& rows) {
  std::vector<Eigen::Vector3d> sensor_t_rays;
  std::ifstream sensor_rays_file(sensor_rays_filename);
  std::string file_line;
  double y, z;
  int line_index = 0;
  while (std::getline(sensor_rays_file, file_line)) {
    std::istringstream line_ss(file_line);
    if (line_index == 0) {
      line_ss >> rows;
    } else {
      line_ss >> y;
      line_ss >> z;
      const Eigen::Vector3d sensor_t_ray(1.0, y, z);
      sensor_t_rays.emplace_back(sensor_t_ray.normalized());
    }
    ++line_index;
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
      LOG(ERROR) << "Failed to get groundtruth pose at timstamp " << std::setprecision(20) << timestamp;
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
                   const std::vector<boost::optional<double>>& depths, const std::string& output_filename,
                   const int rows) {
  std::ofstream output_file;
  output_file.open(output_filename);
  for (const auto timestamp : timestamps) {
    output_file << std::setprecision(20) << timestamp << " ";
  }
  output_file << std::endl;
  const int cols = depths.size() / (timestamps.size() * rows);
  int col = 0;
  for (const auto& depth : depths) {
    if (!depth) {
      output_file << -1 << " ";
    } else {
      output_file << *depth << " ";
    }
    ++col;
    if (col >= cols) {
      std::cout << std::endl;
      col = 0;
    }
  }
  output_file.close();
}
}  // namespace dense_map
