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
#include <localization_common/utilities.h>
#include <localization_common/test_utilities.h>
#include <texture_processing.h>

#include <glog/logging.h>
#include <gtest/gtest.h>

namespace dm = dense_map;
namespace lc = localization_common;

// TODO(rsoussan): put this back!!
const std::string DATA_DIR =
  "/home/rsoussan/isaac/src/dense_map/geometry_mapper/test/data";  // std::string(std::getenv("DATA_DIR"));

TEST(DepthFromMeshTester, Depth) {
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
  // Poses with ray facing mesh at an angle
  {
    auto world_T_sensor = Eigen::Isometry3d::Identity();
    const double yaw_degrees = 45.0;
    world_T_sensor.linear() = lc::RotationFromEulerAngles(yaw_degrees, 0, 0);
    // Create a ray facing along the x axis
    const Eigen::Vector3d sensor_t_ray(1, 0, 0);
    const auto depth = dm::Depth(sensor_t_ray, world_T_sensor, *bvh_tree);
    const double expected_depth = 1.0 / std::cos(yaw_degrees * M_PI / 180.0);
    ASSERT_TRUE(depth != boost::none);
    EXPECT_NEAR(*depth, expected_depth, 1e-6);
  }
}

TEST(DepthFromMeshTester, LoadTimestamp) {
  {
    const std::string filename("123.456_world.txt");
    const auto timestamp = dm::LoadTimestamp(filename);
    EXPECT_NEAR(timestamp, 123.456, 1e-6);
  }
  {
    const std::string filename("123.456_world.txt");
    const auto timestamp = dm::LoadTimestamp(filename, 10);
    EXPECT_NEAR(timestamp, 133.456, 1e-6);
  }
}

TEST(DepthFromMeshTester, LoadSensorRays) {
  const std::string filename(DATA_DIR + "/sensor_rays.csv");
  const auto sensor_rays = dm::LoadSensorRays(filename);
  EXPECT_MATRIX_NEAR(sensor_rays[0], Eigen::Vector3d(1, 1.23, 4.56).normalized(), 1e-6);
  EXPECT_MATRIX_NEAR(sensor_rays[1], Eigen::Vector3d(1, 7.888999, 99999.111222).normalized(), 1e-6);
}

TEST(DepthFromMeshTester, LoadTimestamps) {
  const std::string filename(DATA_DIR + "/timestamps.csv");
  const auto timestamps = dm::LoadTimestamps(filename);
  ASSERT_EQ(timestamps.size(), 3);
  EXPECT_NEAR(timestamps[0], 1.11, 1e-6);
  EXPECT_NEAR(timestamps[1], 2.222, 1e-6);
  EXPECT_NEAR(timestamps[2], 333.3333, 1e-6);
}

TEST(DepthFromMeshTester, LoadTimestampsAndPoses) {
  const std::string directory_name(DATA_DIR);
  std::vector<lc::Time> timestamps;
  std::vector<Eigen::Isometry3d> poses;
  dm::LoadTimestampsAndPoses(directory_name, "nav_cam", timestamps, poses);
  ASSERT_EQ(timestamps.size(), 3);
  ASSERT_EQ(poses.size(), 3);
  {
    EXPECT_NEAR(timestamps[0], 1234.22, 1e-6);
    EXPECT_MATRIX_NEAR(poses[0], Eigen::Isometry3d::Identity(), 1e-6);
  }
  {
    EXPECT_NEAR(timestamps[1], 313131.99, 1e-6);
    EXPECT_MATRIX_NEAR(
      poses[1], lc::Isometry3d(Eigen::Vector3d(-1.1, -0.22, 300.3), lc::RotationFromEulerAngles(45, 45, 45)), 1e-6);
  }
  {
    EXPECT_NEAR(timestamps[2], 22222.1119, 1e-6);
    EXPECT_MATRIX_NEAR(poses[2], lc::Isometry3d(Eigen::Vector3d(1, 2.22, -3.33), Eigen::Matrix3d::Identity()), 1e-6);
  }
}

TEST(DepthFromMeshTester, LoadTimestampsAndPosesWithOffset) {
  const std::string directory_name(DATA_DIR);
  std::vector<lc::Time> timestamps;
  std::vector<Eigen::Isometry3d> poses;
  const double offset = -0.113;
  dm::LoadTimestampsAndPoses(directory_name, "nav_cam", timestamps, poses, Eigen::Isometry3d::Identity(), offset);
  ASSERT_EQ(timestamps.size(), 3);
  ASSERT_EQ(poses.size(), 3);
  {
    EXPECT_NEAR(timestamps[0], 1234.22 + offset, 1e-6);
    EXPECT_MATRIX_NEAR(poses[0], Eigen::Isometry3d::Identity(), 1e-6);
  }
  {
    EXPECT_NEAR(timestamps[1], 313131.99 + offset, 1e-6);
    EXPECT_MATRIX_NEAR(
      poses[1], lc::Isometry3d(Eigen::Vector3d(-1.1, -0.22, 300.3), lc::RotationFromEulerAngles(45, 45, 45)), 1e-6);
  }
  {
    EXPECT_NEAR(timestamps[2], 22222.1119 + offset, 1e-6);
    EXPECT_MATRIX_NEAR(poses[2], lc::Isometry3d(Eigen::Vector3d(1, 2.22, -3.33), Eigen::Matrix3d::Identity()), 1e-6);
  }
}

TEST(DepthFromMeshTester, LoadTimestampsAndPosesWithSensorOffset) {
  const std::string directory_name(DATA_DIR);
  std::vector<lc::Time> timestamps;
  std::vector<Eigen::Isometry3d> poses;
  const auto poses_sensor_T_sensor = lc::RandomIsometry3d();
  dm::LoadTimestampsAndPoses(directory_name, "nav_cam", timestamps, poses, poses_sensor_T_sensor);
  ASSERT_EQ(timestamps.size(), 3);
  ASSERT_EQ(poses.size(), 3);
  {
    EXPECT_NEAR(timestamps[0], 1234.22, 1e-6);
    EXPECT_MATRIX_NEAR(poses[0], Eigen::Isometry3d::Identity() * poses_sensor_T_sensor, 1e-6);
  }
  {
    EXPECT_NEAR(timestamps[1], 313131.99, 1e-6);
    EXPECT_MATRIX_NEAR(poses[1],
                       lc::Isometry3d(Eigen::Vector3d(-1.1, -0.22, 300.3), lc::RotationFromEulerAngles(45, 45, 45)) *
                         poses_sensor_T_sensor,
                       1e-6);
  }
  {
    EXPECT_NEAR(timestamps[2], 22222.1119, 1e-6);
    EXPECT_MATRIX_NEAR(
      poses[2], lc::Isometry3d(Eigen::Vector3d(1, 2.22, -3.33), Eigen::Matrix3d::Identity()) * poses_sensor_T_sensor,
      1e-6);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
