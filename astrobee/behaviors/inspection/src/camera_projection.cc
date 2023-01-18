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
// TODO(mgouveia): look into this, seems like FrustumPlanes does not have the s parameter
// #include <mapper/linear_algebra.h>

#include <cmath>

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
  CameraView::CameraView(std::string cam_name, float f, float n)  : f_(f), n_(n) {
    cam_name_ = cam_name;

    cfg_cam_.AddFile("cameras.config");
    if (!cfg_cam_.ReadFiles())
      ROS_FATAL("Failed to read config files.");

    // Create a transform buffer to listen for transforms
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(tf_buffer_));

    Eigen::Matrix3d cam_mat;

    config_reader::ConfigReader::Table camera(&cfg_cam_, cam_name.c_str());

    // Read in distorted image size.
    if (!camera.GetInt("width", &W_))
      fprintf(stderr, "Could not read camera width.");
    if (!camera.GetInt("height", &H_))
      fprintf(stderr, "Could not read camera height.");

    config_reader::ConfigReader::Table vector(&camera, "intrinsic_matrix");
    for (int i = 0; i < 9; i++) {
      if (!vector.GetReal((i + 1), &cam_mat(i / 3, i % 3))) {
        fprintf(stderr, "Failed to read vector intrinsic_matrix.");
        break;
      }
    }

    SetProjectionMatrix(cam_mat);

    // Get relative camera position
    try {
      tf_body_to_cam_ = tf_buffer_.lookupTransform("body", cam_name_, ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("Failed getting transform: %s", ex.what());
    }
  }


  // Return the Projection Matrix
  Eigen::Matrix4d  CameraView::GetProjectionMatrix() {
    return P_;
  }

  // Return the Horizontal Field of View
  double  CameraView::GetHFOV() {
    return 2 * atan(W_ / (2 * fx_));
  }

  // Return the Vertical Field of View
  double  CameraView::GetVFOV() {
    return 2 * atan(H_ / (2 * fy_));
  }

  // Return the Horizontal Field of View
  double  CameraView::GetH() {
    return H_;
  }

  // Return the Vertical Field of View
  double  CameraView::GetW() {
    return W_;
  }

  //
  bool CameraView::GetCamXYFromPoint(const geometry_msgs::Pose robot_pose, const geometry_msgs::Point point, int& x,
                                     int& y) {
    // Initialize x,y
    x = 0; y = 0;
    Eigen::Vector4d p;
    p << point.x,
         point.y,
         point.z,
         1;
    tf2::Transform camera_pose = msg_conversions::ros_pose_to_tf2_transform(robot_pose) *
                                 msg_conversions::ros_tf_to_tf2_transform(tf_body_to_cam_.transform);

    // Build the View matrix
    Eigen::Quaterniond R(camera_pose.getRotation().w(),
                         camera_pose.getRotation().x(),
                         camera_pose.getRotation().y(),
                         camera_pose.getRotation().z());      // Rotation Matrix Identity
    Eigen::Vector3d T(camera_pose.getOrigin().x(),
                      camera_pose.getOrigin().y(),
                      camera_pose.getOrigin().z());           // Translation Vector
    Eigen::Matrix4d V;                                        // Transformation Matrix
    V.setIdentity();                                          // Identity to make bottom row 0,0,0,1
    V.block<3, 3>(0, 0) = R.normalized().toRotationMatrix();;
    V.block<3, 1>(0, 3) = T;
    // Transform point
    Eigen::Vector4d q = P_ * V.inverse() * p;

    x = static_cast<int>(((q(0) / q(3)) + 1) * W_ / 2);
    y = static_cast<int>(((q(1) / q(3)) + 1) * H_ / 2);

    if (q(0) / q(3) < -1 ||   // the point lies beyond the left border of the screen
        q(0) / q(3) >  1 ||   // the point lies beyond the right border of the screen
        q(1) / q(3) < -1 ||   // the point lies beyond the bottom border of the screen
        q(1) / q(3) >  1 ||   // the point lies beyond the top border of the screen
        q(2) / q(3) < -1 ||   // the point lies beyond the near plane of the camera,
                              //     i.e., the point is behind the camera or too close for the camera to see.
        q(2) / q(3) >  1) {   // the point lies beyond the far plane of the camera,
                              //     i.e., the point is too far away for the camera to see
      if (debug_) {
        ROS_DEBUG_STREAM(V.inverse() * p);
        ROS_DEBUG_STREAM("VisibilityConstraint T pos" << camera_pose.getOrigin().x() << " "
                                    << camera_pose.getOrigin().y() << " " << camera_pose.getOrigin().z());
        ROS_DEBUG_STREAM("VisibilityConstraint T quat"
                         << camera_pose.getRotation().w() << " " << camera_pose.getRotation().x() << " "
                         << camera_pose.getRotation().y() << " " << camera_pose.getRotation().z());
        ROS_DEBUG_STREAM("VisibilityConstraint p " << point.x << " " << point.y << " " << point.z);
        ROS_DEBUG_STREAM("VisibilityConstraint q " << q(0) / q(3) << " " << q(1) / q(3) << " " << q(2) / q(3));

        Eigen::Matrix4d corners_near;
        corners_near << 1, -1, 1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, 1, 1;
        Eigen::Matrix4d p_near = P_.inverse() * corners_near;
        ROS_DEBUG_STREAM("p_near 1 " << p_near(0, 0) / p_near(3, 0) << " " << p_near(1, 0) / p_near(3, 0) << " "
                                     << p_near(2, 0) / p_near(3, 0));
        ROS_DEBUG_STREAM("p_near 2 " << p_near(0, 1) / p_near(3, 1) << " " << p_near(1, 1) / p_near(3, 1) << " "
                                     << p_near(2, 1) / p_near(3, 1));
        ROS_DEBUG_STREAM("p_near 3 " << p_near(0, 2) / p_near(3, 2) << " " << p_near(1, 2) / p_near(3, 2) << " "
                                     << p_near(2, 2) / p_near(3, 2));
        ROS_DEBUG_STREAM("p_near 4 " << p_near(0, 3) / p_near(3, 3) << " " << p_near(1, 3) / p_near(3, 3) << " "
                                     << p_near(2, 3) / p_near(3, 3));

        Eigen::Matrix4d corners_far;
        corners_far << 1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
        Eigen::Matrix4d p_far = P_.inverse() * corners_far;
        ROS_DEBUG_STREAM("p_far 1 " << p_far(0, 0) / p_far(3, 0) << " " << p_far(1, 0) / p_far(3, 0) << " "
                                    << p_far(2, 0) / p_far(3, 0));
        ROS_DEBUG_STREAM("p_far 2 " << p_far(0, 1) / p_far(3, 1) << " " << p_far(1, 1) / p_far(3, 1) << " "
                                    << p_far(2, 1) / p_far(3, 1));
        ROS_DEBUG_STREAM("p_far 3 " << p_far(0, 2) / p_far(3, 2) << " " << p_far(1, 2) / p_far(3, 2) << " "
                                    << p_far(2, 2) / p_far(3, 2));
        ROS_DEBUG_STREAM("p_far 4 " << p_far(0, 3) / p_far(3, 3) << " " << p_far(1, 3) / p_far(3, 3) << " "
                                    << p_far(2, 3) / p_far(3, 3));
      }
      // Eliminate point
      return false;
    }


    return true;
  }


  bool CameraView::GetCamXYFromPoint(const geometry_msgs::Point point, int& x, int& y) {
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
    return GetCamXYFromPoint(msg_conversions::ros_transform_to_ros_pose(robot_pose.transform), point, x, y);
  }

  // Get 3D point from camera pixel location and point cloud
  bool CameraView::GetPointFromXYD(const sensor_msgs::PointCloud2 pCloud, const int x, const int y,
                                   geometry_msgs::Point& point) {
    // Convert from u (column / width), v (row/height) to position in array
    // where X,Y,Z data starts
    int arrayPosition = x*pCloud.row_step + y*pCloud.point_step;

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
    CameraView depth_cam(depth_cam_name + "_cam", f_, n_);
    depth_cam.debug_ = true;

    // Establish where the corners are in the image
    // Be aware that at this point the target has been confirmed to be fully within the selected
    // inspection camera, so even if the GetCamXYFromPoint function returns false it just means that it is
    // outside view because the fov of the depth camera might be smaller than the fov of the selected
    // inspection camera
    tf2::Transform target_transform = msg_conversions::ros_pose_to_tf2_transform(point);
    tf2::Transform p1, p2, p3, p4;
    std::vector<int> vert_x{0, 0, 0, 0}, vert_y{0, 0, 0, 0};
    p1 = target_transform * tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(size_x, size_y, 0));
    if (!depth_cam.GetCamXYFromPoint(msg_conversions::tf2_transform_to_ros_pose(p1).position, vert_x[0], vert_y[0])) {
      ROS_WARN_STREAM("Point p1 outside depth cam view");
    }
    p2 = target_transform * tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(size_x, -size_y, 0));
    if (!depth_cam.GetCamXYFromPoint(msg_conversions::tf2_transform_to_ros_pose(p2).position, vert_x[1], vert_y[1])) {
      ROS_WARN_STREAM("Point p2 outside depth cam view");
    }
    p3 = target_transform * tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(-size_x, size_y, 0));
    if (!depth_cam.GetCamXYFromPoint(msg_conversions::tf2_transform_to_ros_pose(p3).position, vert_x[2], vert_y[2])) {
      ROS_WARN_STREAM("Point p3 outside depth cam view");
    }
    p4 = target_transform * tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(-size_x, -size_y, 0));
    if (!depth_cam.GetCamXYFromPoint(msg_conversions::tf2_transform_to_ros_pose(p4).position, vert_x[3], vert_y[3])) {
      ROS_WARN_STREAM("Point p4 outside depth cam view");
    }

    // Debug messages
    ROS_DEBUG_STREAM("target p1 " << p1.getOrigin().x() << " " << p1.getOrigin().y() << " " << p1.getOrigin().z());
    ROS_DEBUG_STREAM("target p2 " << p2.getOrigin().x() << " " << p2.getOrigin().y() << " " << p2.getOrigin().z());
    ROS_DEBUG_STREAM("target p3 " << p3.getOrigin().x() << " " << p3.getOrigin().y() << " " << p3.getOrigin().z());
    ROS_DEBUG_STREAM("target p4 " << p4.getOrigin().x() << " " << p4.getOrigin().y() << " " << p4.getOrigin().z());

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

    // Project target points from depth image to 3D space + calculate average
    geometry_msgs::Point new_point;
    geometry_msgs::Point sum_point;
    int points_counter = 0;
    for (int depth_cam_x = 0; depth_cam_x < W_; ++depth_cam_x) {
      for (int depth_cam_y = 0; depth_cam_y < H_; ++depth_cam_y) {
        if (InsideTarget(vert_x, vert_y, depth_cam_x, depth_cam_y)) {
          depth_cam.GetPointFromXYD(*msg, depth_cam_x, depth_cam_y, new_point);
          points_counter += 1;
          sum_point.x += new_point.x;
          sum_point.y += new_point.y;
          sum_point.z += new_point.z;
        }
      }
    }
    ROS_DEBUG_STREAM("Sum target using haz cam interception: " << sum_point.x / points_counter << " "
                                                               << sum_point.y / points_counter << " "
                                                               << sum_point.x / points_counter);

    // Calculate distance between estimated target and camera
    geometry_msgs::TransformStamped tf_depth_cam_to_cam;
    try {
      tf_depth_cam_to_cam = tf_buffer_.lookupTransform(depth_cam_name + "_cam", cam_name_,
                                                              ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("Failed getting transform: %s", ex.what());
      return false;
    }

    return sqrt((tf_depth_cam_to_cam.transform.translation.x - sum_point.x / points_counter)
                  * (tf_depth_cam_to_cam.transform.translation.x - sum_point.x / points_counter)
              + (tf_depth_cam_to_cam.transform.translation.y - sum_point.y / points_counter)
                  * (tf_depth_cam_to_cam.transform.translation.y - sum_point.y / points_counter)
              + (tf_depth_cam_to_cam.transform.translation.z - sum_point.z / points_counter)
                  * (tf_depth_cam_to_cam.transform.translation.z - sum_point.z / points_counter));
  }

  // Define the projection matrix based on camera parameters using the pinhole model
  // Note that this definition considers low distortion
  bool CameraView::SetProjectionMatrix(Eigen::Matrix3d cam_mat) {
    // Get camera parameters
    float s, cx, cy;

    // Read in focal length, optical offset and skew
    fx_ = cam_mat(0, 0);
    fy_ = cam_mat(1, 1);
    s  = cam_mat(0, 1);
    cx = cam_mat(0, 2);
    cy = cam_mat(1, 2);

    // Build projection matrix
    P_ << 2 * fx_ / W_,      0,                 0,                      0,
          2 * s / W_,        2 * fy_ / H_,      0,                      0,
          2 * (cx / W_) - 1, 2 * (cy / H_) - 1, -(f_ + n_) / (f_ - n_), 2 * (f_ * n_) / (f_ - n_),
          0,                 0,                 -1,                     0;

    return true;
  }

  // Draw the camera frustum using a marker array for rviz visualization
  void CameraView::DrawCameraFrustum(const geometry_msgs::Pose robot_pose, ros::Publisher &publisher) {
    tf2::Transform camera_pose = msg_conversions::ros_pose_to_tf2_transform(robot_pose) *
                                 msg_conversions::ros_tf_to_tf2_transform(tf_body_to_cam_.transform);

    // Build the View matrix
    Eigen::Quaterniond R = msg_conversions::ros_to_eigen_quat(
      msg_conversions::tf2_quat_to_ros_quat(camera_pose.getRotation()));  // Rotation Matrix Identity
    Eigen::Vector3d T = msg_conversions::ros_point_to_eigen_vector(
                            msg_conversions::tf2_transform_to_ros_pose(camera_pose).position);  // Translation Vector
    Eigen::Matrix4d V;                                        // Transformation Matrix
    V.setIdentity();                                          // Identity to make bottom row 0,0,0,1
    V.block<3, 3>(0, 0) = R.normalized().toRotationMatrix();;
    V.block<3, 1>(0, 3) = T;

    Eigen::Matrix4d corners_near;
    corners_near <<  1,  1, -1, -1, 1, -1, -1,  1, -1, -1, -1, -1, 1,  1,  1,  1;
    Eigen::Matrix4d p_near = V * P_.inverse() * corners_near;


    Eigen::Matrix4d corners_far;
    corners_far <<  1,  1, -1, -1, 1, -1, -1,  1, 1, 1, 1, 1, 1,  1,  1,  1;
    Eigen::Matrix4d p_far = V * P_.inverse() * corners_far;

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
      p.x = p_near(0, i) / p_near(3, i);
      p.y = p_near(1, i) / p_near(3, i);
      p.z = p_near(2, i) / p_near(3, i);
      marker.points.push_back(p);

      p.x = p_near(0, (i + 1) % n) / p_near(3, (i + 1) % n);
      p.y = p_near(1, (i + 1) % n) / p_near(3, (i + 1) % n);
      p.z = p_near(2, (i + 1) % n) / p_near(3, (i + 1) % n);
      marker.points.push_back(p);
    }
    // Add far points
    for (int i = 0; i < n; ++i) {
      p.x = p_far(0, i) / p_far(3, i);
      p.y = p_far(1, i) / p_far(3, i);
      p.z = p_far(2, i) / p_far(3, i);
      marker.points.push_back(p);

      p.x = p_far(0, (i + 1) % n) / p_far(3, (i + 1) % n);
      p.y = p_far(1, (i + 1) % n) / p_far(3, (i + 1) % n);
      p.z = p_far(2, (i + 1) % n) / p_far(3, (i + 1) % n);
      marker.points.push_back(p);
    }
    // Add conn points
    for (int i = 0; i < n; ++i) {
      p.x = p_far(0, i) / p_far(3, i);
      p.y = p_far(1, i) / p_far(3, i);
      p.z = p_far(2, i) / p_far(3, i);
      marker.points.push_back(p);

      p.x = p_near(0, i) / p_near(3, i);
      p.y = p_near(1, i) / p_near(3, i);
      p.z = p_near(2, i) / p_near(3, i);
      marker.points.push_back(p);
    }

    // Add arrow for visualization
    msg_visual.markers.push_back(marker);

    // Publish marker message
    publisher.publish(msg_visual);
  }

}  // namespace inspection
