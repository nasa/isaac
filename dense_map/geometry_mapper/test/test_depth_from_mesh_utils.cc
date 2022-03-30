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

#include <depth_from_mesh_utils.h>
#include <texture_processing.h>

#include <gtest/gtest.h>

namespace dm = dense_map;

TEST(DepthFromMeshTester, NoRotationDepths) {
  std::vector<unsigned int> ids{0, 1, 2};
  std::vector<math::Vec3f> vertices;
  // Build a mesh with a single triangule along the yz plane, offset from the x axis by 1
  vertices.emplace_back(dm::eigen_to_vec3f(Eigen::Vector3d(1, -1, 0)));
  vertices.emplace_back(dm::eigen_to_vec3f(Eigen::Vector3d(1, 1, 0)));
  vertices.emplace_back(dm::eigen_to_vec3f(Eigen::Vector3d(1, 1, 1)));
  const auto bvh_tree = BVHTree::create(ids, vertices);
  // Poses with ray facing mesh
  {
    const auto world_T_sensor = Eigen::Isometry3d::Identity();
    // Create a ray facing along the x axis
    const Eigen::Vector3d sensor_t_ray(1, 0, 0);
    const auto depth = dm::Depth(sensor_t_ray, world_T_sensor, *bvh_tree);
    ASSERT_TRUE(depth != boost::none);
    EXPECT_NEAR(*depth, 1, 1e-6);
  }
  {
    auto world_T_sensor = Eigen::Isometry3d::Identity();
    world_T_sensor.translation().x() = -1;
    // Create a ray facing along the x axis
    const Eigen::Vector3d sensor_t_ray(1, 0, 0);
    const auto depth = dm::Depth(sensor_t_ray, world_T_sensor, *bvh_tree);
    ASSERT_TRUE(depth != boost::none);
    EXPECT_NEAR(*depth, 2, 1e-6);
  }
  // Poses with ray facing away from mesh
  {
    const auto world_T_sensor = Eigen::Isometry3d::Identity();
    // Create a ray facing along the negative x axis
    const Eigen::Vector3d sensor_t_ray(-1, 0, 0);
    const auto depth = dm::Depth(sensor_t_ray, world_T_sensor, *bvh_tree);
    EXPECT_TRUE(depth == boost::none);
  }
  {
    const auto world_T_sensor = Eigen::Isometry3d::Identity();
    // Create a ray facing along the y axis
    const Eigen::Vector3d sensor_t_ray(0, 1, 0);
    const auto depth = dm::Depth(sensor_t_ray, world_T_sensor, *bvh_tree);
    EXPECT_TRUE(depth == boost::none);
  }
  {
    const auto world_T_sensor = Eigen::Isometry3d::Identity();
    // Create a ray facing along the z axis
    const Eigen::Vector3d sensor_t_ray(0, 0, 1);
    const auto depth = dm::Depth(sensor_t_ray, world_T_sensor, *bvh_tree);
    EXPECT_TRUE(depth == boost::none);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
