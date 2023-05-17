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

#ifndef PANO_VIEW_MESH_INTERSECT_H_
#define PANO_VIEW_MESH_INTERSECT_H_

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

namespace pano_view {

struct Face {
  int v1, v2, v3;
};

bool ReadOBJ(const std::string filename, std::vector<Eigen::Vector3d> &vertices, std::vector<Face> &faces);

bool intersect(const Eigen::Vector3d origin, const Eigen::Vector3d dir, const std::vector<Eigen::Vector3d> vertices,
               const std::vector<Face> faces, Eigen::Vector3d &intersection);
bool intersectRayMesh(const std::string filename, const Eigen::Vector3d origin, const Eigen::Vector3d dir,
                      Eigen::Vector3d &intersection);

}  // namespace pano_view

#endif  // PANO_VIEW_MESH_INTERSECT_H_
