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
    tf2::Quaternion quat_point;
    double pan, tilt;

    double dist_resolution = cfg_->Get<double>("distance_resolution");
    double angle_resolution_theta = cfg_->Get<double>("angle_resolution_theta") * M_PI / 180.0;
    double angle_resolution_phi = cfg_->Get<double>("angle_resolution_phi") * M_PI / 180.0;
    double max_angle = cfg_->Get<double>("max_angle") * M_PI / 180.0;

    double max_distance = cfg_->Get<double>("max_distance");
    double min_distance = cfg_->Get<double>("min_distance");
    double target_distance  = cfg_->Get<double>("target_distance");

    // Go through all the alternative points in preference order
    for (double r = 0; (r < max_distance - target_distance) ||
                       (r < target_distance - min_distance); r += dist_resolution) {
      for (double theta = 0; theta <= max_angle; theta += angle_resolution_theta) {
        for (double phi = 0; phi < 2 * M_PI; phi += angle_resolution_phi) {
          ROS_ERROR_STREAM("r: " << r << " phi: " << phi << " theta: " << theta);

            tilt = theta * sin(phi);
            pan = - theta * cos(phi);
            quat_point.setRPY(0, tilt, pan);
            point.orientation = msg_conversions::tf2_quat_to_ros_quat(quat_point);



          if ((target_distance + r < max_distance) && r != 0) {   // avoid publishing twice on zero
            // Insert point
            point.position.x = -(target_distance + r) * cos(theta);
            point.position.y = (target_distance + r) * sin(theta) * cos(phi);
            point.position.z = (target_distance + r) * sin(theta) * sin(phi);
            points.poses.push_back(point);
          }
          if (target_distance - r > min_distance) {
            // Insert point
            point.position.x = -(target_distance - r) * cos(theta);
            point.position.y = (target_distance - r) * sin(theta) * cos(phi);
            point.position.z = (target_distance - r) * sin(theta) * sin(phi);
            // tilt = (M_PI - theta) * cos(phi);
            // pan =  (M_PI - theta) * sin(phi);
            // quat_point.setRPY(0, tilt, pan);
            // point.orientation = msg_conversions::tf2_quat_to_ros_quat(quat_point);
            points.poses.push_back(point);
          }
          if (theta == 0)
            break;
        }
      }
    }
    return 0;
  }


}  // namespace inspection
