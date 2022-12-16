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

    // Insert point in the target reference frame
    point.orientation.x = 1;
    point.orientation.y = 0;
    point.orientation.z = 0;
    point.orientation.w = 0;

    // Go through all the alternative points in preference order
    for (double r = 0; (r < max_distance_ - target_distance_) ||
                       (r < target_distance_ - min_distance_); r += dist_resolution_) {
      for (double theta = 0; theta < max_angle_; theta += angle_resolution_) {
        for (double phi = 0; phi < 2*3.14; phi += angle_resolution_) {
              // ROS_ERROR_STREAM("r: " << r << " phi: " << phi << " z: " << theta);
          if ((target_distance_ + r < max_distance_) && r != 0) {   // avoid publishing twice on zero
            // Insert point
            point.position.x = (target_distance_ + r) * sin(theta) * cos(phi);
            point.position.y = (target_distance_ + r) * sin(theta) * sin(phi);
            point.position.z = (target_distance_ + r)  * cos(theta);
            points.poses.push_back(point);
            // Insert point
            if (theta != 0) {   // avoid publishing twice on zero
              point.position.x = (target_distance_ + r) * sin(-theta) * cos(phi);
              point.position.y = (target_distance_ + r) * sin(-theta) * sin(phi);
              point.position.z = (target_distance_ + r)  * cos(-theta);
              points.poses.push_back(point);
            }
          }
          if (target_distance_ - r > min_distance_) {
            // Insert point
            point.position.x = (target_distance_ - r) * sin(theta) * cos(phi);
            point.position.y = (target_distance_ - r) * sin(theta) * sin(phi);
            point.position.z = (target_distance_ - r)  * cos(theta);
            points.poses.push_back(point);
            // Insert point
            if (theta != 0) {   // avoid publishing twice on zero
              point.position.x = (target_distance_ - r) * sin(-theta) * cos(phi);
              point.position.y = (target_distance_ - r) * sin(-theta) * sin(phi);
              point.position.z = (target_distance_ - r)  * cos(-theta);
              points.poses.push_back(point);
            }
          }
          if (theta == 0)
            break;
        }
      }
    }
    return 0;
  }


}  // namespace inspection
