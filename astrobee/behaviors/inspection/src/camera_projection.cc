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
  CameraView::CameraView(std::string cam_name) {
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

    setProjectionMatrix(cam_mat);
  }


  // Return the Projection Matrix
  Eigen::Matrix4d  CameraView::getProjectionMatrix() {
    return P_;
  }

  // Return the Horizontal Field of View
  double  CameraView::getHFOV() {
    return 2 * atan(W_ / (2 * fx_));
  }

  // Return the Vertical Field of View
  double  CameraView::getVFOV() {
    return 2 * atan(H_ / (2 * fy_));
  }

  // Return the Horizontal Field of View
  double  CameraView::getH() {
    return H_;
  }

  // Return the Vertical Field of View
  double  CameraView::getW() {
    return W_;
  }

  bool CameraView::getCamXYFromPoint(const geometry_msgs::Pose robot_pose, const geometry_msgs::Point point, int& x,
                                     int& y) {
    // Get current camera position
    ROS_ERROR_STREAM("VisibilityConstraint cam " << cam_name_);
    geometry_msgs::TransformStamped tf_body_to_cam = tf_buffer_.lookupTransform("body", cam_name_,
                                                            ros::Time(0));

    Eigen::Vector4d p;
    p << point.x,
         point.y,
         point.z,
         1;
    tf2::Transform camera_pose = msg_conversions::ros_pose_to_tf2_transform(robot_pose) *
                                 msg_conversions::ros_tf_to_tf2_transform(tf_body_to_cam.transform);

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

    if (q(0) / q(3) < -1 ||   // the point lies beyond the left border of the screen
        q(0) / q(3) >  1 ||   // the point lies beyond the right border of the screen
        q(1) / q(3) < -1 ||   // the point lies beyond the bottom border of the screen
        q(1) / q(3) >  1 ||   // the point lies beyond the top border of the screen
        q(2) / q(3) < -1 ||   // the point lies beyond the near plane of the camera,
                              //     i.e., the point is behind the camera or too close for the camera to see.
        q(2) / q(3) >  1) {   // the point lies beyond the far plane of the camera,
                              //     i.e., the point is too far away for the camera to see
    if (debug_) {
      ROS_ERROR_STREAM(V.inverse() * p);
    ROS_ERROR_STREAM("VisibilityConstraint T pos" << camera_pose.getOrigin().x() << " "
                                << camera_pose.getOrigin().y() << " " << camera_pose.getOrigin().z());
    ROS_ERROR_STREAM("VisibilityConstraint T quat"
                     << camera_pose.getRotation().w() << " " << camera_pose.getRotation().x() << " "
                     << camera_pose.getRotation().y() << " " << camera_pose.getRotation().z());
    ROS_ERROR_STREAM("VisibilityConstraint p " << point.x << " " << point.y << " " << point.z);
    ROS_ERROR_STREAM("VisibilityConstraint q " << q(0) / q(3) << " " << q(1) / q(3) << " " << q(2) / q(3));

    Eigen::Matrix4d corners_near;
    corners_near << 1, -1, 1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, 1, 1;
    Eigen::Matrix4d p_near = P_.inverse() * corners_near;
    ROS_ERROR_STREAM("p_near 1 " << p_near(0, 0) / p_near(3, 0) << " " << p_near(1, 0) / p_near(3, 0) << " "
                                 << p_near(2, 0) / p_near(3, 0));
    ROS_ERROR_STREAM("p_near 2 " << p_near(0, 1) / p_near(3, 1) << " " << p_near(1, 1) / p_near(3, 1) << " "
                                 << p_near(2, 1) / p_near(3, 1));
    ROS_ERROR_STREAM("p_near 3 " << p_near(0, 2) / p_near(3, 2) << " " << p_near(1, 2) / p_near(3, 2) << " "
                                 << p_near(2, 2) / p_near(3, 2));
    ROS_ERROR_STREAM("p_near 4 " << p_near(0, 3) / p_near(3, 3) << " " << p_near(1, 3) / p_near(3, 3) << " "
                                 << p_near(2, 3) / p_near(3, 3));

    Eigen::Matrix4d corners_far;
    corners_far << 1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    Eigen::Matrix4d p_far = P_.inverse() * corners_far;
    ROS_ERROR_STREAM("p_far 1 " << p_far(0, 0) / p_far(3, 0) << " " << p_far(1, 0) / p_far(3, 0) << " "
                                << p_far(2, 0) / p_far(3, 0));
    ROS_ERROR_STREAM("p_far 2 " << p_far(0, 1) / p_far(3, 1) << " " << p_far(1, 1) / p_far(3, 1) << " "
                                << p_far(2, 1) / p_far(3, 1));
    ROS_ERROR_STREAM("p_far 3 " << p_far(0, 2) / p_far(3, 2) << " " << p_far(1, 2) / p_far(3, 2) << " "
                                << p_far(2, 2) / p_far(3, 2));
    ROS_ERROR_STREAM("p_far 4 " << p_far(0, 3) / p_far(3, 3) << " " << p_far(1, 3) / p_far(3, 3) << " "
                                << p_far(2, 3) / p_far(3, 3));
    }

          // Eliminate point
          return false;
    }


    x = static_cast<int>(((q(0) / q(3)) + 1) * W_ / 2);
    y = static_cast<int>(((q(1) / q(3)) + 1) * W_ / 2);

    return true;
  }
  bool CameraView::getCamXYFromPoint(const geometry_msgs::Point point, int& x, int& y) {
    // Get current camera position
    ROS_ERROR_STREAM("getCamXYFromPoint get current pose");
    geometry_msgs::TransformStamped robot_pose = tf_buffer_.lookupTransform("world", "body",
                                                            ros::Time(0), ros::Duration(2.0));
    ROS_ERROR_STREAM("got getCamXYFromPoint get current pose");


    return getCamXYFromPoint(msg_conversions::ros_transform_to_ros_pose(robot_pose.transform), point, x, y);
  }




  bool CameraView::getPointFromXYD(const sensor_msgs::PointCloud2 pCloud, const int x, const int y,
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

  double CameraView::getDistanceFromTarget(const geometry_msgs::Pose point, std::string depth_cam_name) {
    ROS_ERROR_STREAM_ONCE("getDistanceFromTarget");
    // Create depth cam camera model
    CameraView depth_cam(depth_cam_name);
    depth_cam.debug_ = true;


    // Get target image coordinate
    int depth_cam_x, depth_cam_y;
    double depth_cam_d;
    if (!depth_cam.getCamXYFromPoint(point.position, depth_cam_x, depth_cam_y)) {
      ROS_ERROR_STREAM("point outside haz cam view ");
      return -1;
    }
    ROS_ERROR_STREAM_ONCE("getDistanceFromPoint getCamXYFromPoint " << depth_cam_x << " " << depth_cam_y);

    // Get most recent depth message
    std::string cam_prefix = TOPIC_HARDWARE_PICOFLEXX_PREFIX;
    std::string cam_suffix = TOPIC_HARDWARE_PICOFLEXX_SUFFIX;

    boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;
    msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
      cam_prefix + "haz" + cam_suffix, ros::Duration(3.5));  // Wait enought time to accomodate simulation
    ROS_ERROR_STREAM("TOPIC: " << cam_prefix << "haz" << cam_suffix);
    if (msg == NULL) {
      ROS_ERROR_STREAM("No point clound message received from " << depth_cam_name);
      return -1;
    }



    // Project target estimate to 3D space
    geometry_msgs::Point new_point;
    depth_cam.getPointFromXYD(*msg, depth_cam_x, depth_cam_y, new_point);
    ROS_ERROR_STREAM_ONCE("getDistanceFromPoint getPointFromXYD " << new_point.x << " " << new_point.y << " "
                                                                  << new_point.x);

    // Calculate distance between estimated target and camera
    geometry_msgs::TransformStamped tf_img_cam_to_world = tf_buffer_.lookupTransform(depth_cam_name, cam_name_,
                                                            ros::Time(0));

    return sqrt((tf_img_cam_to_world.transform.translation.x - new_point.x)
                  * (tf_img_cam_to_world.transform.translation.x - new_point.x)
              + (tf_img_cam_to_world.transform.translation.y - new_point.y)
                  * (tf_img_cam_to_world.transform.translation.y - new_point.y)
              + (tf_img_cam_to_world.transform.translation.z - new_point.z)
                  * (tf_img_cam_to_world.transform.translation.z - new_point.z));
  }

  double CameraView::getDistanceFromCenter(std::string depth_cam_name) {
    // Create depth cam camera model
    CameraView depth_cam(depth_cam_name);


    // Get most recent depth message
    std::string cam_prefix = TOPIC_HARDWARE_PICOFLEXX_PREFIX;
    std::string cam_suffix = TOPIC_HARDWARE_PICOFLEXX_SUFFIX;

    boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;
    msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
                      cam_prefix + "haz" + cam_suffix, ros::Duration(0.5));
    if (msg == NULL) {
      ROS_INFO("No point clound message received");
      return -1;
    }


    // Calculate mean position of camera center points
    geometry_msgs::Point new_point;
    geometry_msgs::Point average_middle;
    average_middle.x = 0.0; average_middle.y = 0.0; average_middle.z = 0.0;
    int range_h = 20, range_w = 20;
    for (int depth_cam_x = depth_cam.getW() / 2 - range_h / 2; depth_cam_x < depth_cam.getW() / 2 + range_h / 2;
         ++depth_cam_x) {
      for (int depth_cam_y = depth_cam.getH() / 2 - range_w / 2; depth_cam_y < depth_cam.getH() / 2 + range_w / 2;
           ++depth_cam_y) {
        depth_cam.getPointFromXYD(*msg, depth_cam_x, depth_cam_y, new_point);
        average_middle.x = average_middle.x + new_point.x / (range_h * range_w);
        average_middle.y = average_middle.y + new_point.x / (range_h * range_w);
        average_middle.z = average_middle.z + new_point.x / (range_h * range_w);
      }
    }

    // Calculate distance between estimated target and camera
    geometry_msgs::TransformStamped tf_img_cam_to_world = tf_buffer_.lookupTransform("world", cam_name_,
                                                            ros::Time(0));

    return sqrt((tf_img_cam_to_world.transform.translation.x - average_middle.x)
                  * (tf_img_cam_to_world.transform.translation.x - new_point.x)
              + (tf_img_cam_to_world.transform.translation.y - average_middle.y)
                  * (tf_img_cam_to_world.transform.translation.y - average_middle.y)
              + (tf_img_cam_to_world.transform.translation.z - average_middle.z)
                  * (tf_img_cam_to_world.transform.translation.z - average_middle.z));
  }


  bool CameraView::setProjectionMatrix(Eigen::Matrix3d cam_mat) {
    // Get camera parameters
    float s, cx, cy;
    // Read in focal length, optical offset and skew
    fx_ = cam_mat(0, 0);
    fy_ = cam_mat(1, 1);
    s  = cam_mat(0, 1);
    cx = cam_mat(0, 2);
    cy = cam_mat(1, 2);

    // Build projection matrix
    float farmnear = max_distance_ - min_distance_;
    // P_ << 2*fx_/W_,    0,           0,                                                                      0,
    //       2*s/W_,      2*fy_/H_,    0,                                                                      0,
    //       2*(cx/W_)-1, 2*(cy/H_)-1, (max_distance_ + min_distance_) / (max_distance_ - min_distance_),      1,
    //       0,           0,           2 * max_distance_ * min_distance_ / (min_distance_ - max_distance_),    0;
    // P_ << 2*fx_/W_,    0,           0,                                                                      0,
    //       2*s/W_,      2*fy_/H_,    0,                                                                      0,
    //       2*(cx/W_)-1, 2*(cy/H_)-1, -(max_distance_ + min_distance_) / (max_distance_ - min_distance_),     -1,
    //       0,           0,           2 * max_distance_ * min_distance_ / (max_distance_ - min_distance_),    0;
    // P_ << 2*fx_/W_,    0,            0,                                            0,
    //       2*s/W_,      2*fy_/H_,     0,                                            0,
    //       2*(cx/W_)-1, 2*(cy/H_)-1, -max_distance_ / (farmnear),                   1,
    //       0,           0,           -min_distance_ * (max_distance_ / (farmnear)), 1;


    P_ << 2*fx_/W_,    0,           0,               0,
          2*s/W_,      2*fy_/H_,    0,               0,
          2*(cx/W_)-1, 2*(cy/H_)-1, 2 / (farmnear), -(max_distance_ + min_distance_) / (farmnear),
          0,           0,           0,               1;

    return true;
  }

}  // namespace inspection

// GLdouble perspMatrix[16]={    2*fx/W,     0,          0,                       0,
                              // 2*s/W,      2*fy/H,     0,                       0,
                              // 2*(cx/W)-1, 2*(cy/H)-1, (zmax+zmin)/(zmax-zmin), 1,
                              // 0,          0,          2*zmax*zmin/(zmin-zmax), 0};

// GLdouble perspMatrix[16]={    2*fx/w,     0,           0,                     0,
                              // 0,          2*fy/h,      0,                     0,
                              // 2*(cx/w)-1, 2*(cy/h)-1, -(far+near)/(far-near),-1,
                              // 0,          0,          -2*far*near/(far-near), 0};
