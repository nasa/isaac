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

#include "pano_view/point_cloud_intersect.h"


namespace pano_view {

double intersect(const Eigen::Vector3d origin, const Eigen::Vector3d dir,
                 const pcl::PointCloud<pcl::PointXYZ> point_cloud, Eigen::Vector3d& intersection) {
  double min_dist = std::numeric_limits<double>::max();
  double dist_point = -1;
  int i = 0;
  for (const auto& pcl_point : point_cloud.points) {
    Eigen::Vector3d point(pcl_point.x, pcl_point.y, pcl_point.z);
    i++;
    Eigen::Vector3d v = point - origin;
    double dot = v.dot(dir);
    if (dot <= 0) {
      continue;
    }
    double dist = (point - origin).cross(dir).norm();

    if (dist < min_dist) {
      min_dist = dist;
      intersection = point;
      dist_point = min_dist;
    }
  }

  return dist_point;
}

bool intersectRayPointCloud(const pcl::PointCloud<pcl::PointXYZ> point_cloud, const Eigen::Vector3d origin,
                            const Eigen::Vector3d dir, Eigen::Vector3d& intersection) {
  double distance;
  if ((distance = intersect(origin, dir, point_cloud, intersection)) > 0) {
    std::cout << "Point Cloud Point to vector distance: " << distance << std::endl;
  } else {
    return false;
  }
  return true;
}

}  // namespace pano_view
