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
#include <msg_conversions/msg_conversions.h>
#include <config_reader/config_reader.h>
#include <ff_util/ff_flight.h>
#include <isaac_util/isaac_names.h>

// Point Cloud
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// Software messages
#include <visualization_msgs/MarkerArray.h>
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
#include <string>
#include <map>

/**
 * \ingroup beh
 */
namespace inspection {

/*
  This class provides camera functionality that allows us
  to project the 3D point into the camera frame and the
  other way around. It automatically reads the camera
  parameters from the config files based on the camera
  name, such that no setup is necessary.
*/
class CameraView {
 public:
  // Constructor
  explicit CameraView(std::string cam_name, float f = 1.0, float n = 0.19);

  Eigen::Matrix4d GetProjectionMatrix();
  double GetHFOV();
  double GetVFOV();

  double GetH();
  double GetW();

  // Gets the points x y where the point is in the image. If outside the image, then it will return false
  // If the robot pose is not specified, it's considered to be the current one
  bool GetCamXYFromPoint(const geometry_msgs::Pose robot_pose, const geometry_msgs::Point point, int &x, int &y);
  bool GetCamXYFromPoint(const geometry_msgs::Point point, int &x, int &y);

  bool GetPointFromXYD(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &point);

  double GetDistanceFromTarget(const geometry_msgs::Pose point, std::string depth_cam_name,
                                double size_x, double size_y);

  void DrawCameraFrustum(const geometry_msgs::Pose robot_pose, ros::Publisher &publisher);

  bool debug_ = false;
  float f_;
  float n_;

 protected:
  bool SetProjectionMatrix(Eigen::Matrix3d cam_mat);
  bool InsideTarget(std::vector<int> vert_x, std::vector<int> vert_y, int test_x, int test_y);

 private:
  std::string cam_name_;
  config_reader::ConfigReader cfg_cam_;
  int W_, H_;
  float fx_, fy_;
  Eigen::Matrix4d P_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::TransformStamped tf_body_to_cam_;

 public:
  // This fixes the Eigen aligment issue
  // http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*
  This class provides the high-level logic that allows the freeflyer to
  define the inspection poses for the different survey types.
  It calls the survey generator and evaluates plans regarding:

  * Visibility constraints (anomaly inspection only)
  * Keepout and Keepin zones
  * Obstacle map

  It returns a vector of inspection poses. "In the case of an anomaly inspection,
  if the move action fails due to planning or an unmapped obstacle, it saves the
  sorted alternatives such that replanning isn't necessary.
  It also constains functions that allow inspection visualization.
*/
class Inspection {
 public:
  // Constructor
  Inspection(ros::NodeHandle* nh, ff_util::ConfigServer* cfg);
  // Read parameters from config server
  void ReadParam();

  // Remove head of segment if planing failed
  bool RemoveInspectionPose();
  // Get te head of the inspection poses segment
  geometry_msgs::PoseArray GetCurrentInspectionPose();
  bool NextInspectionPose();
  bool RedoInspectionPose();
  geometry_msgs::PoseArray GetInspectionPoses();

  // Get distance from camera to target
  double GetDistanceToTarget();

  // Generate the supported inspection methods
  bool GenerateAnomalySurvey(geometry_msgs::PoseArray &points_anomaly);
  bool GenerateGeometrySurvey(geometry_msgs::PoseArray &points_geometry);
  bool GeneratePanoramaSurvey(geometry_msgs::PoseArray &points_panorama);
  bool GenerateVolumetricSurvey(geometry_msgs::PoseArray &points_volume);

 protected:
  // Ensure all clients are connected
  void ConnectedCallback();
  // Timeout on a zone check request
  void CheckZonesTimeoutCallback();
  // Timeout on a map check request
  void CheckMapTimeoutCallback();

  // Checks the given points agains whether the target is visible
  // from a camera picture
  bool VisibilityConstraint(geometry_msgs::PoseArray &points, tf2::Transform target_transform);
  bool PointInsideCuboid(geometry_msgs::Point const& x,
                         geometry_msgs::Vector3 const& cubemin,
                         geometry_msgs::Vector3 const& cubemax);
  bool ZonesConstraint(geometry_msgs::PoseArray &points);
  bool ObstaclesConstraint(geometry_msgs::PoseArray &points);
  // This function transforms the points from the camera rf to the body rf
  bool TransformList(geometry_msgs::PoseArray points_in, geometry_msgs::PoseArray &points_out,
                      tf2::Transform target_transform);

  // Draws the possible inspection poses
  void DrawPoseMarkers(geometry_msgs::PoseArray &points,
                            ros::Publisher &publisher);
  // Draws visibility frostum projection
  void DrawCameraFrustum();

  // This function generates a sorted list based on the max viewing angle and resolution
  bool GenerateSortedList(geometry_msgs::PoseArray &points);

 private:
  ff_util::FreeFlyerServiceClient<ff_msgs::GetZones> client_z_;
  ff_util::FreeFlyerServiceClient<ff_msgs::GetMap> client_o_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // General inspection variables
  std::string mode_;
  int inspection_counter_;

  geometry_msgs::PoseArray goal_;                 // Vector containing inspection goals
  std::vector<geometry_msgs::PoseArray> points_;  // Vector containing inspection poses
  tf2::Quaternion target_to_cam_rot_;

  // Camera Projection functions
  std::string curr_camera_;
  std::map<std::string, CameraView> cameras_;

  // Parameter clients
  ff_util::ConfigServer *cfg_;

  // Inspection parameters
  double horizontal_fov_;
  double aspect_ratio_;
  double target_size_x_;
  double target_size_y_;
  std::string depth_cam_;

  // Panorame parameters
  bool auto_fov_;
  double pan_min_;
  double pan_max_;
  double tilt_min_;
  double tilt_max_;
  double h_fov_;
  double v_fov_;
  double att_tol_;
  double overlap_;

  // Publish Markers
  ros::Publisher pub_targets_;
  ros::Publisher pub_markers_;
  ros::Publisher pub_cam_;

 public:
  // This fixes the Eigen aligment issue
  // http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}  // namespace inspection
#endif  // INSPECTION_INSPECTION_H_
