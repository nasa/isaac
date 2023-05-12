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

#ifndef PANO_VIEW_POINT_CLOUD_INTERSECT_H_
#define PANO_VIEW_POINT_CLOUD_INTERSECT_H_

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

namespace pano_view {

double intersect(const Eigen::Vector3d origin, const Eigen::Vector3d dir,
                 const pcl::PointCloud<pcl::PointXYZ> point_cloud, Eigen::Vector3d& intersection);
bool intersectRayPointCloud(const pcl::PointCloud<pcl::PointXYZ> point_cloud, const Eigen::Vector3d origin,
                            const Eigen::Vector3d dir, Eigen::Vector3d& intersection);

}  // namespace pano_view

#endif  // PANO_VIEW_POINT_CLOUD_INTERSECT_H_
