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

// Reads an OBJ file and extracts the vertex and face information.
// Inputs
//   filename: The input file name of the OBJ file.
// Outputs
//   vertices: A vector to store the vertex information.
//   faces: A vector to store the vertice indexes that forme a face.
// Returns True if the OBJ file is successfully read, false otherwise.
bool ReadOBJ(const std::string filename, std::vector<Eigen::Vector3d> &vertices, std::vector<Face> &faces);

// Intersects a ray defined by an origin and direction with a mesh model represented by a vector of vertices and faces.
// Inputs
//   origin: The origin of the ray.
//   dir: The direction of the ray.
//   vertices: A vector of vertices representing the mesh model.
//   faces: A vector to store the vertice indexes that forme a face.
// Outputs
//   intersections: A vector to store the intersection point.
// Returns True if at least one intersection is found, false otherwise.
bool intersect(const Eigen::Vector3d origin, const Eigen::Vector3d dir, const std::vector<Eigen::Vector3d> vertices,
               const std::vector<Face> faces, Eigen::Vector3d &intersection);


// Reads OBJ file and intersects a ray defined by an origin and direction
// Inputs
//   filename: The input file name of the OBJ file.
//   origin: The origin of the ray.
//   dir: The direction of the ray.
// Outputs
//   intersections: A vector to store the intersection points.
// Returns True if at least one intersection is found, false otherwise.
bool intersectRayMesh(const std::string filename, const Eigen::Vector3d origin, const Eigen::Vector3d dir,
                      Eigen::Vector3d &intersection);

}  // namespace pano_view

#endif  // PANO_VIEW_MESH_INTERSECT_H_
