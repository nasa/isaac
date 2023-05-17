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

#ifndef INSPECTION_CAMERA_PROJECTION_H_
#define INSPECTION_CAMERA_PROJECTION_H_

// Standard ROS includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// TF2 support
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Shared project includes
#include <ff_common/ff_names.h>
#include <msg_conversions/msg_conversions.h>
#include <config_reader/config_reader.h>
#include <isaac_util/isaac_names.h>

// Point Cloud
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// Software messages
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

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
  This class provides camera functionality that allows us to project the 3D point into
  the camera frame and the other way around. It automatically reads the camera parameters
  from the config files based on the camera name, such that no setup is necessary.
*/
class CameraView {
 public:
  // Constructor for Camera View taking in inputs:
  // cam_name: name of the camera that it used to read from config file the parameters
  // f: far clip, maximum camera distance (m)
  // n: near clip, minimum camera distance (m)
  // cam_transform: transform from body->camera, useful for offline applications where tf is not available
  explicit CameraView(const std::string cam_name, const float f = 2.0, const float n = 0.19,
                      const geometry_msgs::Transform::ConstPtr cam_transform = NULL);

  // Return the Projection Matrix
  Eigen::Matrix4d GetProjectionMatrix();

  // Return the Horizontal Field of View
  double GetHFOV();
  // Return the Vertical Field of View
  double GetVFOV();

  // Return Camera pixel Height
  double GetH();
  // Return Camera pixel Width
  double GetW();

  // Set Camera pixel Height
  void SetH(const double H);
  // Set Camera pixel Width
  void SetW(const double W);

  // Build the View Matrix based on robot pose
  bool BuildViewMatrix(const geometry_msgs::Pose robot_pose, Eigen::Matrix4d &V);


  // Returns the vector that contains the camera position and the point in the far clip that corresponds
  // to the pixel index provided
  bool GetVectorFromCamXY(const geometry_msgs::Pose robot_pose, const int x, const int y,
                                     Eigen::Vector3d &vector);

  // Gets the points x y where the point is in the image. If outside the image, then it will return false
  // If the robot pose is not specified, it's considered to be the current one
  bool GetCamXYFromPoint(const geometry_msgs::Pose robot_pose, const geometry_msgs::Point point, int &x, int &y);
  bool GetCamXYFromPoint(const geometry_msgs::Point point, int &x, int &y);

  // Get 3D point from camera pixel location and point cloud
  bool GetPointFromXYD(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &point);

  // Get the distance from the camera to the target using depth camera information
  double GetDistanceFromTarget(const geometry_msgs::Pose point, std::string depth_cam_name,
                                double size_x, double size_y);

  // Draw the camera frustum using a marker array for rviz visualization
  void DrawCameraFrustum(const geometry_msgs::Pose robot_pose, ros::Publisher &publisher);

  bool debug_ = false;
  float f_;
  float n_;

 protected:
  // Checks if a point is inside a poligon, in this case with 4 sides.
  bool InsideTarget(std::vector<int> vert_x, std::vector<int> vert_y, int test_x, int test_y);
  // Define the projection matrix based on camera parameters using the pinhole model
  bool SetProjectionMatrix(Eigen::Matrix3d cam_mat);

 private:
  std::string cam_name_;
  config_reader::ConfigReader cfg_cam_;
  int W_, H_;
  float fx_, fy_;
  Eigen::Matrix4d P_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::Transform tf_body_to_cam_;

 public:
  // This fixes the Eigen aligment issue
  // http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}  // namespace inspection
#endif  // INSPECTION_CAMERA_PROJECTION_H_
