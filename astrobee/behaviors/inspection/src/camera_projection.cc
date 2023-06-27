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
#include <inspection/camera_projection.h>
// TODO(mgouveia): look into this, seems like FrustumPlanes does not have the s parameter
// #include <mapper/linear_algebra.h>

#include <math.h>
#include <cmath>

/**
 * \ingroup beh
 */
namespace inspection {
/*
  This class provides camera functionality that allows us to project the 3D point into
  the camera frame and the other way around. It automatically reads the camera parameters
  from the config files based on the camera name, such that no setup is necessary.
*/
CameraView::CameraView(const camera::CameraParameters & params, const float f, const float n,
                       const geometry_msgs::Transform::ConstPtr cam_transform)
    : camera::CameraModel(params), f_(f), n_(n) {

  // Get relative camera position
  if (cam_transform == NULL) {
    // Create a transform buffer to listen for transforms
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));
    try {
      tf_body_to_cam_ = msg_conversions::ros_to_eigen_transform(
        tf_buffer_.lookupTransform("body", cam_name_, ros::Time(0), ros::Duration(1.0)).transform);
    } catch (tf2::TransformException& ex) {
      ROS_ERROR("Failed getting transform: %s", ex.what());
    }
  } else {
    tf_body_to_cam_ = msg_conversions::ros_to_eigen_transform(*cam_transform);
  }
}

// Gets the points x y where the point is in the image. If outside the image, then it will return false
bool CameraView::GetCamXYFromPoint(const Eigen::Affine3d robot_pose, const Eigen::Vector3d point,
                                    int& x, int& y) {
  // Initialize x,y
  x = 0; y = 0;
  // Set Camera transform to current position
  SetTransform(robot_pose * tf_body_to_cam_);
  // Check if target is in the fov
  if (IsInFov(point)) {
    Eigen::Vector2d c;
    GetParameters().Convert<camera::UNDISTORTED, camera::DISTORTED>(ImageCoordinates(point), &c);
    // Convert back to the 0->size format
    c = c + (GetParameters().GetUndistortedHalfSize()).cast<double>();
    x = c[0] ; y = c[1];
  } else {
    return false;
  }
  return true;
}

// Gets the points x y where the point is in the image. If outside the image, then it will return false
// If the robot pose is not specified, it's considered to be the current one
bool CameraView::GetCamXYFromPoint(const Eigen::Vector3d point, int& x, int& y) {
  // Initialize x,y
  x = 0; y = 0;
  // Get current camera position if it's not given
  geometry_msgs::TransformStamped robot_pose;
  try {
    robot_pose = tf_buffer_.lookupTransform("world", "body", ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Failed getting transform: %s", ex.what());
    return false;
  }
  return GetCamXYFromPoint(msg_conversions::ros_to_eigen_transform(robot_pose.transform), point, x, y);
}

// Get 3D point from camera pixel location and point cloud
bool CameraView::GetPointFromXYD(const sensor_msgs::PointCloud2 pCloud, const int u, const int v,
                                 geometry_msgs::Point& point) {
  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  int arrayPosition = v * pCloud.row_step + u * pCloud.point_step;

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset;  // X has an offset of 0
  int arrayPosY = arrayPosition + pCloud.fields[1].offset;  // Y has an offset of 4
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset;  // Z has an offset of 8

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

  // make sure output is valid
  if (std::isnan(X) || std::isnan(Y) || std::isnan(Z)
          || std::isinf(X) || std::isinf(Y) || std::isinf(Z) ||
          (X == 0.0 && Y == 0.0 && Z == 0.0))
    return false;

  // put data into the point p
  point.x = X;
  point.y = Y;
  point.z = Z;

  return true;
}

// Checks if a point is inside a poligon, in this case with 4 sides.
// Explanation of the method and example implementation in:
// https://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
bool CameraView::InsideTarget(std::vector<int> vert_x, std::vector<int> vert_y, int test_x, int test_y) {
  int i, j;
  bool c = false;
  int n_vert = 4;
  for (i = 0, j = n_vert-1; i < n_vert; j = i++) {
    if (((vert_y[i] > test_y) != (vert_y[j] > test_y)) &&
     (test_x < (vert_x[j] - vert_x[i]) * (test_y - vert_y[i]) / (vert_y[j]-vert_y[i]) + vert_x[i]) )
       c = !c;
  }
  return c;
}

  // Get the distance from the camera to the target using depth camera information
  double CameraView::GetDistanceFromTarget(const geometry_msgs::Pose point, std::string depth_cam_name, double size_x,
                                           double size_y) {
    // Create depth cam camera model
    static camera::CameraParameters depth_cam_params(&cfg_cam_, (depth_cam_name + "_cam").c_str());
    CameraView depth_cam(depth_cam_params, f_, n_);

    // Get most recent depth message
    std::string cam_prefix = TOPIC_HARDWARE_PICOFLEXX_PREFIX;
    std::string cam_suffix = TOPIC_HARDWARE_PICOFLEXX_SUFFIX;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;
    msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
      cam_prefix + depth_cam_name + cam_suffix, ros::Duration(3.5));  // Wait enought time to accomodate simulation
    if (msg == NULL) {
      ROS_ERROR_STREAM("No point clound message received from " << depth_cam_name << "_cam"
                                                                << " using topic: " << cam_prefix << depth_cam_name
                                                                << cam_suffix);
      return -1;
    }

    // Confirm that the image size matches the camera parameter size
    Eigen::Vector2i depth_cam_size = depth_cam_params.GetDistortedSize();
    if (depth_cam_size[0] != msg->width && depth_cam_size[1] != msg->height) {
      ROS_ERROR_STREAM("Size of the camera in config does not match actual camera!!");
    }

    // Establish where the corners are in the image
    // Be aware that at this point the target has been confirmed to be fully within the selected
    // inspection camera, so even if the GetCamXYFromPoint function returns false it just means that it is
    // outside view because the fov of the depth camera might be smaller than the fov of the selected
    // inspection camera
    Eigen::Affine3d target_transform = msg_conversions::ros_pose_to_eigen_transform(point);
    Eigen::Affine3d p1, p2, p3, p4;
    std::vector<int> vert_x{0, 0, 0, 0}, vert_y{0, 0, 0, 0};
    p1 = target_transform * Eigen::Translation3d(size_x, size_y, 0);
    if (!depth_cam.GetCamXYFromPoint(p1.translation(), vert_x[0], vert_y[0])) {
      ROS_WARN_STREAM("Point p1 outside depth cam view");
    }
    p2 = target_transform * Eigen::Translation3d(size_x, -size_y, 0);
    if (!depth_cam.GetCamXYFromPoint(p2.translation(), vert_x[1], vert_y[1])) {
      ROS_WARN_STREAM("Point p2 outside depth cam view");
    }
    p3 = target_transform * Eigen::Translation3d(-size_x, size_y, 0);
    if (!depth_cam.GetCamXYFromPoint(p3.translation(), vert_x[2], vert_y[2])) {
      ROS_WARN_STREAM("Point p3 outside depth cam view");
    }
    p4 = target_transform * Eigen::Translation3d(-size_x, -size_y, 0);
    if (!depth_cam.GetCamXYFromPoint(p4.translation(), vert_x[3], vert_y[3])) {
      ROS_WARN_STREAM("Point p4 outside depth cam view");
    }

    // Debug messages
    ROS_DEBUG_STREAM("target p1 " << p1.translation()[0] << " " << p1.translation()[1] << " " << p1.translation()[2]);
    ROS_DEBUG_STREAM("target p2 " << p2.translation()[0] << " " << p2.translation()[1] << " " << p2.translation()[2]);
    ROS_DEBUG_STREAM("target p3 " << p3.translation()[0] << " " << p3.translation()[1] << " " << p3.translation()[2]);
    ROS_DEBUG_STREAM("target p4 " << p4.translation()[0] << " " << p4.translation()[1] << " " << p4.translation()[2]);

    // Project target points from depth image to 3D space + calculate average
    geometry_msgs::Point new_point;
    geometry_msgs::Point sum_point;
    int points_counter = 0;
    for (int depth_cam_x = 0; depth_cam_x < msg->width; ++depth_cam_x) {
      for (int depth_cam_y = 0; depth_cam_y < msg->height; ++depth_cam_y) {
        if (InsideTarget(vert_x, vert_y, depth_cam_x, depth_cam_y) &&
          depth_cam.GetPointFromXYD(*msg, depth_cam_x, depth_cam_y, new_point)) {
          ROS_DEBUG_STREAM("u:" << depth_cam_x << " v:" << depth_cam_y
                                << " (" << new_point.x << ", " << new_point.y << ", " << new_point.z << ")");
          points_counter += 1;
          sum_point.x += new_point.x;
          sum_point.y += new_point.y;
          sum_point.z += new_point.z;
        }
      }
    }
    if (points_counter == 0) {
      ROS_ERROR("No target points in depth cam image");
      return false;
    }
    ROS_DEBUG_STREAM("Average target using haz cam interception: " << sum_point.x / points_counter << " "
                                                               << sum_point.y / points_counter << " "
                                                               << sum_point.z / points_counter);

    // Calculate distance between estimated target and camera
    geometry_msgs::TransformStamped tf_depth_cam_to_cam;
    try {
      tf_depth_cam_to_cam = tf_buffer_.lookupTransform(depth_cam_name + "_cam", cam_name_,
                                                              ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("Failed getting transform: %s", ex.what());
      return false;
    }
    // Return the camera's z axis distance
    return abs(tf_depth_cam_to_cam.transform.translation.z - sum_point.z / points_counter);
  }

  // Draw the camera frustum using a marker array for rviz visualization
  void CameraView::DrawCameraFrustum(const geometry_msgs::Pose robot_pose, ros::Publisher &publisher) {
    Eigen::Affine3d camera_pose = msg_conversions::ros_pose_to_eigen_transform(robot_pose) *
                                 tf_body_to_cam_;

    // Figure out the vector to the corners
    std::vector<Eigen::Vector3d> v(4);
    v[0] = Ray(-GetParameters().GetDistortedHalfSize()[0], -GetParameters().GetDistortedHalfSize()[1]);
    v[1] = Ray(-GetParameters().GetDistortedHalfSize()[0], GetParameters().GetDistortedHalfSize()[1]);
    v[2] = Ray(GetParameters().GetDistortedHalfSize()[0], -GetParameters().GetDistortedHalfSize()[1]);
    v[3] = Ray(GetParameters().GetDistortedHalfSize()[0], GetParameters().GetDistortedHalfSize()[1]);

    visualization_msgs::MarkerArray msg_visual;

    // Initialize marker message
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markers;
    // Fill in marker properties
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    // With of the line
    marker.scale.x = 0.01;
    // Define color
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 1.0;

    marker.id = 0;

    // Add near points
    int n = 4;
    geometry_msgs::Point p;
    for (int i = 0; i < n; ++i) {
      marker.points.push_back(msg_conversions::eigen_to_ros_point(camera_pose.translation() + n_ * v[i]));
      marker.points.push_back(msg_conversions::eigen_to_ros_point(camera_pose.translation() + n_ * v[(i + 1) % n]));
    }
    // Add far points
    for (int i = 0; i < n; ++i) {
      marker.points.push_back(msg_conversions::eigen_to_ros_point(camera_pose.translation() + f_ * v[i]));
      marker.points.push_back(msg_conversions::eigen_to_ros_point(camera_pose.translation() + f_ * v[(i + 1) % n]));
    }
    // Add conn points
    for (int i = 0; i < n; ++i) {
      marker.points.push_back(msg_conversions::eigen_to_ros_point(camera_pose.translation() + n_ * v[i]));
      marker.points.push_back(msg_conversions::eigen_to_ros_point(camera_pose.translation() + f_ * v[i]));
    }

    // Add arrow for visualization
    msg_visual.markers.push_back(marker);

    // Publish marker message
    publisher.publish(msg_visual);
  }

}  // namespace inspection
