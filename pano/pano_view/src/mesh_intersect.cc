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



#include "pano_view/mesh_intersect.h"


namespace pano_view {

bool intersect(const Eigen::Vector3d origin, const Eigen::Vector3d dir, const std::vector<Vertex> vertices,
               const std::vector<Face> faces, Eigen::Vector3d intersection) {
  double min_dist = std::numeric_limits<double>::max();
  bool found = false;
  for (const auto& face : faces) {
    const auto& v1 = vertices[face.v1 - 1];
    const auto& v2 = vertices[face.v2 - 1];
    const auto& v3 = vertices[face.v3 - 1];
    Eigen::Vector3d e1(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);
    Eigen::Vector3d e2(v3.x - v1.x, v3.y - v1.y, v3.z - v1.z);
    Eigen::Vector3d p = dir.cross(e2);
    double det = e1.dot(p);
    if (fabs(det) < std::numeric_limits<double>::epsilon()) {
      continue;
    }
    Eigen::Vector3d t = origin - Eigen::Vector3d(v1.x, v1.y, v1.z);
    double u = t.dot(p) / det;
    if (u < 0.0 || u > 1.0) {
      continue;
    }
    Eigen::Vector3d q = t.cross(e1);
    double v = dir.dot(q) / det;
    if (v < 0.0 || u + v > 1.0) {
      continue;
    }
    double dist = (t.dot(q)) / det;
    if (dist < min_dist) {
      min_dist = dist;
      intersection = origin + dir * dist;
      found = true;
    }
  }
  return found;
}

bool intersectRayMesh(const std::string filename, const Eigen::Vector3d origin, const Eigen::Vector3d dir,
                      Eigen::Vector3d intersection) {
  // Open mesh file
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Could not open file." << std::endl;
    return false;
  }
  // Initialize the vertices and mesh vectors
  std::vector<Vertex> vertices;
  std::vector<Face> faces;
  // Read the file
  std::string line;
  while (getline(file, line)) {
    if (line.substr(0, 2) == "v ") {
      Vertex vertex;
      sscanf(line.c_str(), "v %lf %lf %lf", &vertex.x, &vertex.y, &vertex.z);
      vertices.push_back(vertex);
    } else if (line.substr(0, 2) == "f ") {
      Face face;
      sscanf(line.c_str(), "f %d %d %d", &face.v1, &face.v2, &face.v3);
      faces.push_back(face);
    }
  }
  file.close();

  // Calculate intersection point
  if (intersect(origin, dir, vertices, faces, intersection)) {
    std::cout << "Intersection point: (" << intersection.x() << ", " << intersection.y() << ", " << intersection.z()
              << ")" << std::endl;
  } else {
    std::cout << "No intersection found." << std::endl;
    return false;
  }
  return true;
}

}  // namespace pano_view
