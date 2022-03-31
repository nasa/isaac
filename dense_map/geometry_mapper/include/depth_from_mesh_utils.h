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

#ifndef DEPTH_FROM_MESH_UTILS_H_
#define DEPTH_FROM_MESH_UTILS_H_

#include <localization_common/pose_interpolater.h>
#include <localization_common/time.h>

#include <acc/bvh_tree.h>
#include <mve/mesh.h>
#include <math/vector.h>

#include <Eigen/Geometry>

#include <boost/optional.hpp>

#include <string>
#include <vector>

namespace dense_map {
typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;

boost::optional<double> Depth(const Eigen::Vector3d& sensor_t_ray, const Eigen::Isometry3d& world_T_sensor,
                              const BVHTree& bvh_tree, double min_depth = 1e-3, double max_depth = 1e2);

localization_common::Time LoadTimestamp(const std::string& filename, const double timestamp_offset = 0.0);

void LoadTimestampsAndPoses(const std::string& directory, const std::string& sensor_name,
                            std::vector<localization_common::Time>& timestamps, std::vector<Eigen::Isometry3d>& poses,
                            const Eigen::Isometry3d& poses_sensor_T_sensor = Eigen::Isometry3d::Identity(),
                            const double timestamp_offset = 0.0);

// Assumes each timestamp is on a newline of the file
std::vector<localization_common::Time> LoadTimestamps(const std::string& timestamps_filename);

localization_common::PoseInterpolater MakePoseInterpolater(
  const std::string& directory, const Eigen::Isometry3d& body_T_sensor,
  const std::vector<std::string>& groundtruth_sensor_names,
  const std::vector<Eigen::Isometry3d>& body_T_groundtruth_sensor_vec, const std::vector<double>& timestamp_offsets);

// Assumes each point is sampled from an y, z grid with an x offset of 1.0
// Points are entered as "y z" values with new points on a newline in the file
std::vector<Eigen::Vector3d> LoadSensorRays(const std::string& sensor_rays_filename);

std::shared_ptr<BVHTree> LoadMeshTree(const std::string& mesh_file);

std::vector<boost::optional<double>> GetDepthData(
  const std::vector<localization_common::Time>& timestamps, const std::vector<Eigen::Vector3d>& sensor_t_rays,
  const localization_common::PoseInterpolater& groundtruth_pose_interpolater, const BVHTree& bvh_tree);

void SaveDepthData(const std::vector<localization_common::Time>& timestamps,
                   const std::vector<Eigen::Vector3d>& sensor_t_rays,
                   const std::vector<boost::optional<double>>& depths, const std::string& output_filename);
}  // namespace dense_map

#endif  // DEPTH_FROM_MESH_UTILS_H_
