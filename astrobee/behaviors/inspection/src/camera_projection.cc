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

  bool CameraView::getCamXYFromPoint(const geometry_msgs::Point point, int& x, int& y) {
    // Get current camera position
    geometry_msgs::TransformStamped tf_cam_to_world = tf_buffer_.lookupTransform(cam_name_, "world",
                                                            ros::Time(0));


    // Build the matrix with the points to evaluate
    // Eigen::MatrixXd p(4, 4);
    // p << target_size_x_,  target_size_x_, -target_size_x_, -target_size_x_,
    //      target_size_y_, -target_size_y_,  target_size_y_, -target_size_y_,
    //      0,             0,             0,              0,
    //      1,             1,             1,              1;

    Eigen::Vector4d p;
    p << point.x,
         point.y,
         point.z,
         1;

    // Build the View matrix
    Eigen::Quaterniond R(tf_cam_to_world.transform.rotation.w,
                         tf_cam_to_world.transform.rotation.x,
                         tf_cam_to_world.transform.rotation.x,
                         tf_cam_to_world.transform.rotation.x);     // Rotation Matrix Identity
    Eigen::Vector3d T(tf_cam_to_world.transform.translation.x,
                      tf_cam_to_world.transform.translation.y,
                      tf_cam_to_world.transform.translation.z);     // Translation Vector
    Eigen::Matrix4d V;                                              // Transformation Matrix
    V.setIdentity();                                                // Identity to make bottom row 0,0,0,1
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
          // Eliminate point
          return false;
    }
    x = static_cast<int>(((q(0) / q(3)) + 1) * W_ / 2);
    y = static_cast<int>(((q(1) / q(3)) + 1) * W_ / 2);

    return true;
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

  double CameraView::getPointDistance(const geometry_msgs::Point point, std::string depth_cam_name) {
    // Create depth cam camera model
    CameraView depth_cam(depth_cam_name);


    // Get most recent depth message
    std::string cam_prefix = TOPIC_HARDWARE_PICOFLEXX_PREFIX;
    std::string cam_suffix = TOPIC_HARDWARE_PICOFLEXX_SUFFIX;

    boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;
    msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
                      cam_prefix + depth_cam_name + cam_suffix, ros::Duration(0.5));
    if (msg == NULL) {
      ROS_INFO("No point clound message received");
      return -1;
    }

    // Get target image coordinate
    int depth_cam_x, depth_cam_y;
    double depth_cam_d;
    depth_cam.getCamXYFromPoint(point, depth_cam_x, depth_cam_y);

    // Project target estimate to 3D space
    geometry_msgs::Point new_point;
    depth_cam.getPointFromXYD(*msg, depth_cam_x, depth_cam_y, new_point);



    // Calculate distance between estimated target and camera
    geometry_msgs::TransformStamped tf_img_cam_to_world = tf_buffer_.lookupTransform(cam_name_, "world",
                                                            ros::Time(0));

    return sqrt((tf_img_cam_to_world.transform.translation.x - new_point.x)
                  * (tf_img_cam_to_world.transform.translation.x - new_point.x)
              + (tf_img_cam_to_world.transform.translation.y - new_point.y)
                  * (tf_img_cam_to_world.transform.translation.y - new_point.y)
              + (tf_img_cam_to_world.transform.translation.z - new_point.z)
                  * (tf_img_cam_to_world.transform.translation.z - new_point.z));
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

    P_ << 2*fx_/W_,    0,           0,                                            0,
          2*s/W_,      2*fy_/H_,    0,                                            0,
          2*(cx/W_)-1, 2*(cy/H_)-1, max_distance_ / (farmnear),                   1,
          0,           0,          -min_distance_ * (max_distance_ / (farmnear)), 1;

    return true;
  }

}  // namespace inspection

