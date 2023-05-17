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
bool ReadOBJ(const std::string filename, std::vector<Eigen::Vector3d> &vertices, std::vector<Face> &faces) {
  // Open mesh file
  std::ifstream file(filename.c_str());
  if (!file.is_open()) {
    std::cerr << "Could not open file." << std::endl;
    return false;
  }
  // Read the file
  std::string line;
  while (getline(file, line)) {
    if (line.substr(0, 2) == "v ") {
      Eigen::Vector3d vertex;
      sscanf(line.c_str(), "v %lf %lf %lf", &vertex[0], &vertex[1], &vertex[2]);
      vertices.push_back(vertex);
    } else if (line.substr(0, 2) == "f ") {
      Face face;
      if (sscanf(line.c_str(), "f %d %d %d", &face.v1, &face.v2, &face.v3) != 3) {
        if (sscanf(line.c_str(), "f %d/%*d %d/%*d %d/%*d", &face.v1, &face.v2, &face.v3) != 3) {
          if (sscanf(line.c_str(), "f %d/%*d/%*d %d/%*d/%*d %d/%*d/%*d", &face.v1, &face.v2, &face.v3) != 3) {
            std::cout << "could not read line: " << line << std::endl;
          }
        }
      }
      faces.push_back(face);
    }
  }
  file.close();
  return true;
}

bool intersect(const Eigen::Vector3d origin, const Eigen::Vector3d dir, const std::vector<Eigen::Vector3d> vertices,
               const std::vector<Face> faces, Eigen::Vector3d &intersection) {
  double min_dist = std::numeric_limits<double>::max();
  bool found = false;
  for (const Face& face : faces) {
    // Read vertices in face
    Eigen::Vector3d v1 = vertices[face.v1 - 1];
    Eigen::Vector3d v2 = vertices[face.v2 - 1];
    Eigen::Vector3d v3 = vertices[face.v3 - 1];

    // Calculate the triangle edges
    Eigen::Vector3d e1(v2 - v1);
    Eigen::Vector3d e2(v3 - v1);

    // Normal vector the triangle
    Eigen::Vector3d n = e1.cross(e2);
    // Determinant in the Möller-Trumbore intersection algorithm
    double det = - dir.dot(n);
    if (fabs(det) < std::numeric_limits<double>::epsilon()) {
      continue;
    }

    Eigen::Vector3d ao = origin - v1;
    Eigen::Vector3d dao = ao.cross(dir);

    // Barycentric coordinate of the intersection point along e1.
    double u = e2.dot(dao) / det;
    if (u < 0.0) {
      continue;
    }

    // Barycentric coordinate of the intersection point along e2
    double v = - e1.dot(dao) / det;
    if (v < 0.0 || u + v > 1.0) {
      continue;
    }

    // t Distance from the origin to the intersection point along dir
    double t = ao.dot(n) / det;
    if (t < 0.0) {
      // This means that there is a line intersection but not a ray intersection
      continue;
    }

    if (t < min_dist) {
      min_dist = t;
      intersection = origin + dir * t;
    }
    found = true;
  }
  return found;
}

bool intersectRayMesh(const std::string filename, const Eigen::Vector3d origin, const Eigen::Vector3d dir,
                      Eigen::Vector3d &intersection) {
  // Initialize the vertices and mesh vectors
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Face> faces;
  if (!ReadOBJ(filename, vertices, faces)) {
    return false;
  }

  // Calculate intersection point
  return intersect(origin, dir, vertices, faces, intersection);
}

}  // namespace pano_view
