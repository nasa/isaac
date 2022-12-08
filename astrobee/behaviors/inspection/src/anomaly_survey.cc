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

// Include inspection library header
#include <inspection/inspection.h>
#define PI 3.1415
#define EPS 1e-5
/**
 * \ingroup beh
 */
namespace inspection {
/*
  This library generates close-up anomaly surveys given the relevant parameters
*/

  // This function generates a sorted list based on the max viewing angle and resolution
  bool Inspection::GenerateSortedList(geometry_msgs::PoseArray &points) {
    geometry_msgs::Pose point;

    // Insert point
    point.orientation.x = 1;
    point.orientation.y = 0;
    point.orientation.z = 0;
    point.orientation.w = 0;

    // Go through all the alternative points in preference order
    for (double r = 0; (r < max_distance_ - opt_distance_) ||
                       (r < opt_distance_ - min_distance_); r += dist_resolution_) {
      for (double theta = 0; theta < max_angle_; theta += angle_resolution_) {
        for (double phi = 0; phi < 2*3.14; phi += angle_resolution_) {
              // ROS_ERROR_STREAM("r: " << r << " phi: " << phi << " z: " << theta);
          if ((opt_distance_ + r < max_distance_) && r != 0) {   // avoid publishing twice on zero
            // Insert point
            point.position.x = (opt_distance_ + r) * sin(theta) * cos(phi);
            point.position.y = (opt_distance_ + r) * sin(theta) * sin(phi);
            point.position.z = (opt_distance_ + r)  * cos(theta);
            points_.poses.push_back(point);
            // Insert point
            if (theta != 0) {   // avoid publishing twice on zero
              point.position.x = (opt_distance_ + r) * sin(-theta) * cos(phi);
              point.position.y = (opt_distance_ + r) * sin(-theta) * sin(phi);
              point.position.z = (opt_distance_ + r)  * cos(-theta);
              points_.poses.push_back(point);
            }
          }
          if (opt_distance_ - r > min_distance_) {
            // Insert point
            point.position.x = (opt_distance_ - r) * sin(theta) * cos(phi);
            point.position.y = (opt_distance_ - r) * sin(theta) * sin(phi);
            point.position.z = (opt_distance_ - r)  * cos(theta);
            points_.poses.push_back(point);
            // Insert point
            if (theta != 0) {   // avoid publishing twice on zero
              point.position.x = (opt_distance_ - r) * sin(-theta) * cos(phi);
              point.position.y = (opt_distance_ - r) * sin(-theta) * sin(phi);
              point.position.z = (opt_distance_ - r)  * cos(-theta);
              points_.poses.push_back(point);
            }
          }
          if (theta == 0)
            break;
        }
      }
    }
    return 0;
  }

  // Checks the given points agains whether the target is visible
  // from a camera picture
  // bool Inspection::VisibilityConstraint(geometry_msgs::PoseArray &points) {
  //   // Get camera parameters
  //   Eigen::Matrix3d cam_mat;
  //   float fx, fy, s, cx, cy;
  //   int W, H;

  //   config_reader::ConfigReader::Table camera(&cfg_cam_, points.header.frame_id.c_str());
  //   // Read in distorted image size.
  //   if (!camera.GetInt("width", &W))
  //     fprintf(stderr, "Could not read camera width.");
  //   if (!camera.GetInt("height", &H))
  //     fprintf(stderr, "Could not read camera height.");

  //   config_reader::ConfigReader::Table vector(&camera, "intrinsic_matrix");
  //   for (int i = 0; i < 9; i++) {
  //     if (!vector.GetReal((i + 1), &cam_mat(i / 3, i % 3))) {
  //       fprintf(stderr, "Failed to read vector intrinsic_matrix.");
  //       break;
  //     }
  //   }
  //   // Read in focal length, optical offset and skew
  //   fx = cam_mat(0, 0);
  //   fy = cam_mat(1, 1);
  //   s  = cam_mat(0, 1);
  //   cx = cam_mat(0, 2);
  //   cy = cam_mat(1, 2);

  //   // Build the matrix with the points to evaluate
  //   Eigen::MatrixXd p(4, 4);
  //   p << target_size_x_,  target_size_x_, -target_size_x_, -target_size_x_,
  //        target_size_y_, -target_size_y_,  target_size_y_, -target_size_y_,
  //        0,             0,             0,              0,
  //        1,             1,             1,              1;

  //   // Build projection matrix
  //   float farmnear = max_distance_ - min_distance_;
  //   Eigen::Matrix4d P;
  //   P << 2*fx/W,     0,       0,                                                0,
  //        2*s/W,      2*fy/H,  0,                                                0,
  //        2*(cx/W)-1, 2*(cy/H)-1, max_distance_ / (farmnear),                    1,
  //        0,          0,          -min_distance_ * (max_distance_ / (farmnear)), 1;

  //   // Go through all the points in sorted segment
  //   std::vector<geometry_msgs::Pose>::const_iterator it = points.poses.begin();
  //   while (it != points.poses.end()) {
  //     // Build the View matrix
  //     Eigen::Quaterniond R(1, 0, 0, 0);                                    // Rotation Matrix Identity
  //     Eigen::Vector3d T(it->position.x, it->position.y, it->position.z);   // Translation Vector
  //     Eigen::Matrix4d V;                                                   // Transformation Matrix
  //     V.setIdentity();                                                     // Identity to make bottom row 0,0,0,1
  //     V.block<3, 3>(0, 0) = R.normalized().toRotationMatrix();;
  //     V.block<3, 1>(0, 3) = T;
  //     // Transform point
  //     Eigen::MatrixXd q = P * V.inverse() * p;

  //     bool eliminated = false;
  //     for (int i = 0; i < q.cols(); ++i) {
  //       if (q(0, i)/q(3, i) < -1 ||   // the point lies beyond the left border of the screen
  //           q(0, i)/q(3, i) >  1 ||   // the point lies beyond the right border of the screen
  //           q(1, i)/q(3, i) < -1 ||   // the point lies beyond the bottom border of the screen
  //           q(1, i)/q(3, i) >  1 ||   // the point lies beyond the top border of the screen
  //           q(2, i)/q(3, i) < -1 ||   // the point lies beyond the near plane of the camera,
  //                                     //     i.e., the point is behind the camera or too close for the camera to see.
  //           q(2, i)/q(3, i) >  1) {   // the point lies beyond the far plane of the camera,
  //                                     //     i.e., the point is too far away for the camera to see
  //             // Eliminate point
  //             points.poses.erase(it);
  //             eliminated = true;
  //             break;
  //       }
  //     }
  //     if (!eliminated)
  //       ++it;
  //   }

  //   // Check if there are any points left
  //   if (points.poses.empty())
  //     return false;
  //   else
  //     return true;
  // }





}  // namespace inspection
