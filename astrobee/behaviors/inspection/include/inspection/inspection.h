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

#ifndef INSPECTION_INSPECTION_H_
#define INSPECTION_INSPECTION_H_

// Standard ROS includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// TF2 support
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Shared project includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_service.h>
#include <ff_util/ff_fsm.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>
#include <ff_util/ff_flight.h>
#include <isaac_util/isaac_names.h>

// Point Cloud
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// Software messages
#include <visualization_msgs/MarkerArray.h>
#include <isaac_msgs/InspectionState.h>
#include <ff_msgs/Zone.h>
#include <geometry_msgs/PoseArray.h>

// Services
#include <ff_msgs/SetState.h>
#include <ff_msgs/GetZones.h>
#include <ff_msgs/GetMap.h>

// Actions
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/DockAction.h>
#include <isaac_msgs/ImageInspectionAction.h>
#include <isaac_msgs/InspectionAction.h>

// Eigen for math
#include <Eigen/Dense>

// C++ headers
#include <vector>

/**
 * \ingroup beh
 */
namespace inspection {

/*
  This class provides the high-level logic that allows the freeflyer to
  define the optimal inspection pose. It evaluates:

  * Visibility constraints
  * Keepout and Keepin zones
  * Obstacle map

  It returns a vector of possible inspection poses that can be updated
  in case the move action fails due to planning or unmapped obstacle.
  It also constains functions that allow inspection visualization.
*/
class Inspection {
 public:
  // Constructor
  Inspection(ros::NodeHandle* nh, ff_util::ConfigServer cfg);
  // Generate inspection segment
  bool GenSegment(geometry_msgs::Pose goal);
  // Remove head of segment if planing failed
  bool RemoveInspectionPose();
  // Get te head of the inspection poses segment
  geometry_msgs::Pose GetInspectionPose();

  // Generate the survey for panorama pictures
  void GeneratePanoramaSurvey(geometry_msgs::PoseArray &points_panorama);

 protected:
  // Ensure all clients are connected
  void ConnectedCallback();
  // Timeout on a zone check request
  void CheckZonesTimeoutCallback();
  // Timeout on a map check request
  void CheckMapTimeoutCallback();
  // This function generates a sorted list based on the max viewing angle and resolution
  bool GenerateSortedList(geometry_msgs::PoseArray &points);
  // This function transforms the points from the camera rf to the body rf
  bool TransformList(geometry_msgs::PoseArray points_in, geometry_msgs::PoseArray &points_out,
                      tf2::Transform target_transform);
  // Checks the given points agains whether the target is visible
  // from a camera picture
  bool VisibilityConstraint(geometry_msgs::PoseArray &points);
  bool PointInsideCuboid(geometry_msgs::Point const& x,
                         geometry_msgs::Vector3 const& cubemin,
                         geometry_msgs::Vector3 const& cubemax);
  bool ZonesConstraint(geometry_msgs::PoseArray &points);
  bool ObstaclesConstraint(geometry_msgs::PoseArray &points);
  // Draws the possible inspection poses
  void DrawInspectionPoses(geometry_msgs::PoseArray &points,
                            ros::Publisher &publisher);
  // Draws visibility frostum projection
  void DrawInspectionFrostum();

 private:
  ff_util::FreeFlyerServiceClient<ff_msgs::GetZones> client_z_;
  ff_util::FreeFlyerServiceClient<ff_msgs::GetMap> client_o_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  ros::Publisher pub_;
  tf2::Quaternion vent_to_scicam_rot_;

  geometry_msgs::PoseArray points_;    // Vector containing inspection poses

  // Inspection parameters
  double opt_distance_;
  double dist_resolution_;
  double angle_resolution_;
  double max_angle_;
  double max_distance_;
  double min_distance_;
  double horizontal_fov_;
  double aspect_ratio_;
  double vent_size_x_;
  double vent_size_y_;

  // Publish Markers
  ros::Publisher pub_no_filter_;
  ros::Publisher pub_vis_check_;
  ros::Publisher pub_zones_check_;
  ros::Publisher pub_map_check_;
};

}  // namespace inspection
#endif  // INSPECTION_INSPECTION_H_
