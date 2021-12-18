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

// TODO(oalexan1): Clean these headers.
// TODO(oalexan1): Move some things to utils

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/dynamic_numeric_diff_cost_function.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/autodiff_cost_function.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <ff_common/utils.h>
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/reprojection.h>
#include <config_reader/config_reader.h>
#include <msg_conversions/msg_conversions.h>

#include <dense_map_ros_utils.h>
#include <dense_map_utils.h>

#include <string>
#include <map>
#include <iostream>
#include <fstream>

DEFINE_string(cam1_name, "", "The name of the first camera. Must be 'nav_cam' or 'sci_cam'.");

DEFINE_string(cam2_name, "haz_cam", "The name of the second camera. Must be haz_cam.");

DEFINE_string(cam1_dir, "", "The directory having the corners extracted from cam1 images using Kalibr.");

DEFINE_string(cam2_dir, "", "The directory having the corners extracted from cam2 images using Kalibr.");

DEFINE_string(bag, "", "The ros bag having the calibration data.");

DEFINE_string(haz_cam_points_topic, "/hw/depth_haz/points", "The depth point cloud topic in the bag file.");

DEFINE_double(start, 0.0, "The starting time in seconds from which to process the bag.");

DEFINE_double(duration, -1.0, "Specify how many seconds from the bag to process (default: process the whole bag).");

DEFINE_bool(update_cam1, false, "Update the intrinsics of cam1.");

DEFINE_bool(update_cam2, false, "Update the intrinsics of cam2.");

DEFINE_bool(update_depth_to_image_transform, false,
            "Update the transform to apply to scale correctly the depth camera clouds.");

DEFINE_bool(update_extrinsics, false, "Update the transform from cam2 to cam1.");

DEFINE_string(timestamp_offset_sampling, "",
              "If specified, as 'beg end num', create num samples between beg and end, "
              "and determine the timestamp offset between these cameras as the sample "
              "minimizing the extrinsics error among cam1 and cam2.");

DEFINE_string(cam1_intrinsics_to_float, "",
              "Refine 0 or more of the following intrinsics for cam1: focal_length, "
              "optical_center, distortion. Specify as a quoted list. "
              "For example: 'focal_length optical_center'.");

DEFINE_string(cam2_intrinsics_to_float, "",
              "Refine 0 or more of the following intrinsics for cam2: focal_length, "
              "optical_center, distortion. Specify as a quoted list. "
              "For example: 'focal_length optical_center'.");

DEFINE_int32(num_cam1_focal_lengths, 1,
             "If set to 2, use separate focal lengths along image rows and columns for cam1. Else use the same one.");

DEFINE_int32(num_cam2_focal_lengths, 1,
             "If set to 2, use separate focal lengths along image rows and columns for cam2. Else use the same one.");

DEFINE_int32(num_cam1_iterations, 1000, "How many solver iterations to perform to solve for cam1 intrinsics.");

DEFINE_int32(num_cam2_iterations, 1000, "How many solver iterations to perform to solve for cam2 intrinsics.");

DEFINE_int32(num_extrinsics_iterations, 1000, "How many solver iterations to perform to solve for extrinsics.");

DEFINE_double(robust_threshold, 4.0,
              "Pixel errors much larger than this will be exponentially attenuated "
              "to affect less the cost function. This should not be too low, "
              "as if the initial guess is off, initial pixel errors can be "
              "rather large.");

DEFINE_double(parameter_tolerance, 1e-12, "Stop when the optimization variables change by less than this.");

DEFINE_int32(calib_num_ransac_iterations, 1000,
             "Use in this many RANSAC iterations to find camera poses based on "
             "calibration target measurements.");

DEFINE_int32(calib_ransac_inlier_tolerance, 10, "Use this inlier tolerance (in pixels) to find camera poses.");

DEFINE_double(max_interp_dist, 4.0,
              "If two consecutive camera poses have timestamps that differ more than this,"
              "do not interpolate between them.");

DEFINE_double(max_haz_cam_image_to_depth_timestamp_diff, 0.2,
              "Use depth haz cam clouds that are within this distance in "
              "time from the nearest haz cam intensity image.");

DEFINE_string(output_dir, "",
              "If provided, write here the residuals differences between pixel target "
              "corners and projection into cameras of measured target points) before "
              "and after optimization.");

namespace dense_map {
ceres::LossFunction* GetLossFunction(std::string cost_fun, double th) {
  // Convert to lower-case
  std::transform(cost_fun.begin(), cost_fun.end(), cost_fun.begin(), ::tolower);

  ceres::LossFunction* loss_function = NULL;
  if (cost_fun == "l2")
    loss_function = NULL;
  else if (cost_fun == "huber")
    loss_function = new ceres::HuberLoss(th);
  else if (cost_fun == "cauchy")
    loss_function = new ceres::CauchyLoss(th);
  else if (cost_fun == "l1")
    loss_function = new ceres::SoftLOneLoss(th);
  else
    LOG(FATAL) << "Unknown cost function: " + cost_fun;

  return loss_function;
}

// An error function for finding the best intrinsics of a camera
// by taking n pictures of a calibration target and computing
// the reprojection error for given intrinsics and camera orientation
// relative to the target.
struct IntrinsicsError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IntrinsicsError(Eigen::Vector2d const& pix, Eigen::Vector3d const& xyz, std::vector<int> const& block_sizes,
                  boost::shared_ptr<camera::CameraParameters> orig_cam_params)
      : m_pix(pix), m_xyz(xyz), m_block_sizes(block_sizes), m_orig_cam_params(orig_cam_params) {
    // Sanity check. Note that we can have one or two focal lengths.
    if (m_block_sizes.size() != 4 || (m_block_sizes[0] != 1 && m_block_sizes[0] != 2) || m_block_sizes[1] != 2 ||
        m_block_sizes[2] <= 0 || m_block_sizes[3] != NUM_RIGID_PARAMS) {
      LOG(FATAL) << "IntrinsicsError: The block sizes were not set up properly.\n";
    }
  }

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  // Takes array of arrays as parameters.
  bool operator()(double const* const* parameters, double* residuals) const {
    // Populate the intrinsics

    Eigen::Vector2d focal_vector;
    if (m_block_sizes[0] == 1)
      focal_vector = Eigen::Vector2d(parameters[0][0], parameters[0][0]);  // 1 focal length
    else
      focal_vector = Eigen::Vector2d(parameters[0][0], parameters[0][1]);  // 2 focal lengths

    Eigen::Vector2d optical_center(parameters[1][0], parameters[1][1]);
    Eigen::VectorXd distortion(m_block_sizes[2]);
    for (int i = 0; i < m_block_sizes[2]; i++) distortion[i] = parameters[2][i];

    // Make a deep copy which we will modify
    camera::CameraParameters cam_params = *m_orig_cam_params;

    // Set the current values of the parameters we optimize
    cam_params.SetFocalLength(focal_vector);
    cam_params.SetOpticalOffset(optical_center);
    cam_params.SetDistortion(distortion);

    // The current transform from the world to the camera
    Eigen::Affine3d world_to_cam;
    array_to_rigid_transform(world_to_cam, parameters[3]);

    // Project the measured calibration target corner into the camera
    Eigen::Vector3d cX = world_to_cam * m_xyz;
    Eigen::Vector2d undist_pix = cam_params.GetFocalVector().cwiseProduct(cX.hnormalized());
    Eigen::Vector2d dist_pix;
    cam_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undist_pix, &dist_pix);

    // Compute the residuals
    residuals[0] = dist_pix[0] - m_pix[0];
    residuals[1] = dist_pix[1] - m_pix[1];

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Eigen::Vector2d const& pix, Eigen::Vector3d const& xyz,
                                     std::vector<int> const& block_sizes,
                                     boost::shared_ptr<camera::CameraParameters> orig_cam_params) {
    ceres::DynamicNumericDiffCostFunction<IntrinsicsError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<IntrinsicsError>(
        new IntrinsicsError(pix, xyz, block_sizes, orig_cam_params));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_RESIDUALS);

    // The camera wrapper knows all of the block sizes to add.
    for (size_t i = 0; i < block_sizes.size(); i++) {
      cost_function->AddParameterBlock(block_sizes[i]);
    }
    return cost_function;
  }

 private:
  Eigen::Vector2d m_pix;  // The pixel observation
  Eigen::Vector3d m_xyz;  // The measured position
  std::vector<int> m_block_sizes;
  boost::shared_ptr<camera::CameraParameters> m_orig_cam_params;  // the camera before optimization
};                                                                // End class IntrinsicsError

// An error function for projecting 3D measured points to measured pixels,
// and finding the optimal intrinsics for minimizing the projection error.
// This is used to calibrate the haz_cam intrinsics, to make its depth
// measurements consistent with its intensity image.
struct IntrinsicsDepthError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IntrinsicsDepthError(Eigen::Vector2d const& pix, Eigen::Vector3d const& xyz, std::vector<int> const& block_sizes,
                       boost::shared_ptr<camera::CameraParameters> orig_cam_params)
      : m_pix(pix), m_xyz(xyz), m_block_sizes(block_sizes), m_orig_cam_params(orig_cam_params) {
    // Sanity check. Note that we can have one or two focal lengths.
    if (m_block_sizes.size() != 3 || (m_block_sizes[0] != 1 && m_block_sizes[0] != 2) || m_block_sizes[1] != 2 ||
        m_block_sizes[2] <= 0) {
      LOG(FATAL) << "IntrinsicsError: The block sizes were not set up properly.\n";
    }
  }

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  // Takes array of arrays as parameters.
  bool operator()(double const* const* parameters, double* residuals) const {
    // Populate the intrinsics

    Eigen::Vector2d focal_vector;
    if (m_block_sizes[0] == 1)
      focal_vector = Eigen::Vector2d(parameters[0][0], parameters[0][0]);  // 1 focal length
    else
      focal_vector = Eigen::Vector2d(parameters[0][0], parameters[0][1]);  // 2 focal lengths

    Eigen::Vector2d optical_center(parameters[1][0], parameters[1][1]);
    Eigen::VectorXd distortion(m_block_sizes[2]);
    for (int i = 0; i < m_block_sizes[2]; i++) distortion[i] = parameters[2][i];

    // Make a deep copy which we will modify
    camera::CameraParameters cam_params = *m_orig_cam_params;

    // Set the current values of the parameters we optimize
    cam_params.SetFocalLength(focal_vector);
    cam_params.SetOpticalOffset(optical_center);
    cam_params.SetDistortion(distortion);

    //     std::cout << "focal vector is " << focal_vector.transpose() << std::endl;
    //     std::cout << "optical center is " << optical_center.transpose() << std::endl;
    //     std::cout << "distortion is " << distortion.transpose() << std::endl;

    // Project the xyz point into the camera
    // Note that undist_pix is relative to the center of the image
    // but dist_pix is positive (relative to (0, 0) image corner)
    Eigen::Vector2d undist_pix = cam_params.GetFocalVector().cwiseProduct(m_xyz.hnormalized());
    Eigen::Vector2d dist_pix;
    cam_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undist_pix, &dist_pix);

    //     std::cout << "--dist 0 " << dist_pix[0] << ' ' << m_pix[0] << std::endl;
    //     std::cout << "--dist 1 " << dist_pix[1] << ' ' << m_pix[1] << std::endl;

    residuals[0] = dist_pix[0] - m_pix[0];
    residuals[1] = dist_pix[1] - m_pix[1];

    //     std::cout << "--residual is " << residuals[0] << ' ' << residuals[1] << std::endl;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Eigen::Vector2d const& pix, Eigen::Vector3d const& xyz,
                                     std::vector<int> const& block_sizes,
                                     boost::shared_ptr<camera::CameraParameters> orig_cam_params) {
    ceres::DynamicNumericDiffCostFunction<IntrinsicsDepthError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<IntrinsicsDepthError>(
        new IntrinsicsDepthError(pix, xyz, block_sizes, orig_cam_params));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_RESIDUALS);

    // The camera wrapper knows all of the block sizes to add.
    for (size_t i = 0; i < block_sizes.size(); i++) {
      cost_function->AddParameterBlock(block_sizes[i]);
    }
    return cost_function;
  }

 private:
  Eigen::Vector2d m_pix;  // The pixel observation
  Eigen::Vector3d m_xyz;  // The measured position
  std::vector<int> m_block_sizes;
  boost::shared_ptr<camera::CameraParameters> m_orig_cam_params;  // the camera before optimization
};                                                                // End class IntrinsicsDepthError

// A Ceres cost function solving for extrinsics, so
// cam2_to_cam1_transform. Model the fact that cam1
// and cam2 were acquired at different times.
struct ExtrinsicsError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ExtrinsicsError(Eigen::Vector2d const& target_corner_pixel, Eigen::Vector2d const& target_corner_meas,
                  Eigen::Affine3d const& interp_world_to_cam1, std::vector<int> const& block_sizes,
                  boost::shared_ptr<camera::CameraParameters> cam2_params)
      : m_target_corner_pixel(target_corner_pixel),
        m_target_corner_meas(target_corner_meas),
        m_interp_world_to_cam1(interp_world_to_cam1),
        m_block_sizes(block_sizes),
        m_cam2_params(cam2_params) {}

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  bool operator()(double const* const* parameters, double* residuals) const {
    try {
      if (m_block_sizes.size() != 1 || m_block_sizes[0] != NUM_RIGID_PARAMS)
        LOG(FATAL) << "ExtrinsicsError: The block sizes were not set up properly.\n";

      Eigen::Affine3d cam2_to_cam1_transform;
      array_to_rigid_transform(cam2_to_cam1_transform, parameters[0]);

      Eigen::Affine3d world_to_cam2 = cam2_to_cam1_transform.inverse() * m_interp_world_to_cam1;

      Eigen::Vector3d X;
      X[0] = m_target_corner_meas[0];
      X[1] = m_target_corner_meas[1];
      X[2] = 0;

      Eigen::Vector3d cX0 = world_to_cam2 * X;
      Eigen::Vector2d undist_pix0 = m_cam2_params->GetFocalVector().cwiseProduct((cX0).hnormalized());
      Eigen::Vector2d dist_pix0;
      m_cam2_params->Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undist_pix0, &dist_pix0);

      residuals[0] = dist_pix0[0] - m_target_corner_pixel[0];
      residuals[1] = dist_pix0[1] - m_target_corner_pixel[1];

      if (std::isnan(residuals[0]) || std::isnan(residuals[1])) {
        residuals[0] = 0.0;
        residuals[1] = 0.0;
      }
    } catch (std::exception const& e) {
      // If anything goes wrong just set the residuals to 0
      residuals[0] = 0.0;
      residuals[1] = 0.0;
    }

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Eigen::Vector2d const& target_corner_pixel,
                                     Eigen::Vector2d const& target_corner_meas,
                                     Eigen::Affine3d const& interp_world_to_cam1, std::vector<int> const& block_sizes,
                                     boost::shared_ptr<camera::CameraParameters> cam2_params) {
    ceres::DynamicNumericDiffCostFunction<ExtrinsicsError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<ExtrinsicsError>(
        new ExtrinsicsError(target_corner_pixel, target_corner_meas, interp_world_to_cam1, block_sizes, cam2_params));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_RESIDUALS);

    // The camera wrapper knows all of the block sizes to add.
    for (size_t i = 0; i < block_sizes.size(); i++) {
      cost_function->AddParameterBlock(block_sizes[i]);
    }
    return cost_function;
  }

 private:
  Eigen::Vector2d m_target_corner_pixel;
  Eigen::Vector2d m_target_corner_meas;
  Eigen::Affine3d m_interp_world_to_cam1;
  std::vector<int> m_block_sizes;
  boost::shared_ptr<camera::CameraParameters> m_cam2_params;
};  // End class ExtrinsicsError

void read_measurements(std::string const& filename, std::vector<std::vector<double>>& meas) {
  // std::cout << "Reading: " << filename << std::endl;
  std::ifstream cr(filename.c_str());
  std::string line;
  meas.clear();
  while (std::getline(cr, line)) {
    double val;
    std::vector<double> vals;
    std::istringstream is(line);
    while (is >> val) vals.push_back(val);
    if (vals.size() != 5) LOG(FATAL) << "Each line of " << filename << " must have 5 entries.";
    meas.push_back(vals);
  }
}

// Read all calibration target corners detected with Kalibr
void read_target_corners(  // Inputs
  double start, double duration, std::string const& data_dir,
  // Outputs
  std::map<double, std::vector<std::vector<double>>>& target_corners) {
  // Wipe the inputs
  target_corners.clear();
  double beg_timestamp = -1.0;

  std::vector<std::string> corners_files;
  ff_common::ListFiles(data_dir, "txt", &corners_files);
  for (size_t it = 0; it < corners_files.size(); it++) {
    std::string corners_file = corners_files[it];

    // Initialize the beg timestamp
    double timestamp = dense_map::fileNameToTimestamp(corners_file);

    if (timestamp <= 0) {
      std::cout << "Ignoring file: " << corners_file << std::endl;
      continue;  // likely the wrong kind of file
    }

    if (beg_timestamp < 0.0) beg_timestamp = timestamp;

    if (timestamp - beg_timestamp < start) continue;  // did not yet reach the desired starting time

    if (duration > 0 && timestamp - beg_timestamp > start + duration) break;  // past the desired ending time

    std::vector<std::vector<double>> corners;
    read_measurements(corners_file, corners);
    if (corners.empty()) continue;

    target_corners[timestamp] = corners;
  }
}

// Find the depth cam measurement at each target point (in that
// camera's coordinate system). The target points come from the
// intensity image, while the depth measurements from the depth cloud,
// which have slightly different timestamps. So some bracketing is
// required.
void read_depth_cam_meas(  // Inputs
  std::string const& bag_file, std::string const& haz_cam_points_topic,
  std::map<double, std::vector<std::vector<double>>> const& cam2_target_corners,
  // Outputs
  std::map<double, std::vector<Eigen::Vector3d>>& target_corners_depth) {
  // Wipe the outputs
  target_corners_depth.clear();

  // It is convenient to have the timestamps for the intensity in a vector
  std::vector<double> cam2_timestamps;
  for (auto it = cam2_target_corners.begin(); it != cam2_target_corners.end(); it++) {
    // They better be in increasing order
    if (!cam2_timestamps.empty() && cam2_timestamps.back() >= it->first)
      LOG(FATAL) << "Expecting timestamps to increase.";

    cam2_timestamps.push_back(it->first);
  }

  if (cam2_timestamps.empty()) {
    std::cout << "Could not read any target corners." << std::endl;
    return;
  }

  // Iterate over the depth cam clouds
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(haz_cam_points_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // Keep track of the index of the haz cam image whose timestamp
  // is less but close to the timestamp of the haz cam cloud.
  size_t left_haz_cam_image_index = 0;

  // Process each haz cam cloud by bracketing it between haz cam
  // image timestamps and finding the image that is closest in time.
  for (rosbag::MessageInstance const m : view) {
    sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (!pc_msg) continue;
    double cloud_time = pc_msg->header.stamp.toSec();

    if (cloud_time < cam2_timestamps[left_haz_cam_image_index]) {
      // We are too early in the bag, not yet at the point where we have
      // data to process
      continue;
    }

    if (cloud_time >= cam2_timestamps.back()) {
      // We passed the last timestamp for which we have data to process
      break;
    }

    // Bracket the cloud time between two depth cam image timestamps
    for (size_t cam2_cid = left_haz_cam_image_index; cam2_cid + 1 < cam2_timestamps.size(); cam2_cid++) {
      double left_haz_cam_image_time = cam2_timestamps[cam2_cid + 0];
      double right_haz_cam_image_time = cam2_timestamps[cam2_cid + 1];

      // Save for the future this index
      left_haz_cam_image_index = cam2_cid;

      if (cloud_time >= right_haz_cam_image_time) {
        // Continue advancing in time with cam2_cid.
        continue;
      }

      // So now left_haz_cam_image_time <= cloud_time < right_haz_cam_image_time
      if (cloud_time < left_haz_cam_image_time || cloud_time >= right_haz_cam_image_time)
        LOG(FATAL) << "Book-keeping failure.";

      size_t image_cid = 0;
      if (std::abs(left_haz_cam_image_time - cloud_time) <= std::abs(right_haz_cam_image_time - cloud_time)) {
        image_cid = cam2_cid;
      } else {
        image_cid = cam2_cid + 1;
      }

      // The haz cam image should be normally quite close to the haz
      // cam cloud. Else the results won't be accurate.
      double diff_time = std::abs(cam2_timestamps[image_cid] - cloud_time);
      if (diff_time > FLAGS_max_haz_cam_image_to_depth_timestamp_diff) {
        std::cout << "Found a gap of " << diff_time << " seconds between haz cam intensity and "
                  << "depth timestamps. This is too big. Skipping this frame.\n";
        break;
      }

      pcl::PointCloud<pcl::PointXYZ> pc;
      dense_map::msgToPcl(pc_msg, pc);

      if (static_cast<int>(pc.points.size()) != pc_msg->width * pc_msg->height)
        LOG(FATAL) << "Extracted point cloud size does not agree with original size.";

      double best_image_timestamp = cam2_timestamps[image_cid];
      auto it = cam2_target_corners.find(best_image_timestamp);
      if (it == cam2_target_corners.end())
        LOG(FATAL) << "Cannot locate target corners at desired timestamp.";

      // Will push here one depth measurement for each target corners.
      // Invalid ones will be (0, 0, 0).
      target_corners_depth[best_image_timestamp].clear();

      std::vector<std::vector<double>> const& target_corners = it->second;
      for (size_t pt_iter = 0; pt_iter < target_corners.size(); pt_iter++) {
        Eigen::Vector2d target_corner_pixel;
        // The value  target_corners[pt_iter][0] is the corner id
        // Then come the world measurements, and pixel measurements
        target_corner_pixel[0] = target_corners[pt_iter][3];  // projection into camera x
        target_corner_pixel[1] = target_corners[pt_iter][4];  // projection into camera y

        // Round to find the index in the depth cloud
        size_t col = round(target_corner_pixel[0]);
        size_t row = round(target_corner_pixel[1]);

        if (col < 0 || row < 0 || col > pc_msg->width || row > pc_msg->height) {
          LOG(FATAL) << "Invalid target corner pixel: " << target_corner_pixel.transpose() << std::endl;
        }

        Eigen::Vector3d depth_pt;
        depth_pt << 0.0, 0.0, 0.0;

        // Maybe after rounding we got out of range, so have to check
        if (col < pc_msg->width && row < pc_msg->height) {
          size_t count = row * pc_msg->width + col;
          depth_pt[0] = pc.points[count].x;
          depth_pt[1] = pc.points[count].y;
          depth_pt[2] = pc.points[count].z;
        }

        target_corners_depth[best_image_timestamp].push_back(depth_pt);
      }

      // Bracketing was successful, and the current cloud was processed
      break;
    }
  }  // End iterating over the bag
}

// Create the vector timestamp offset samples. If none are specified
// on the command line, use only the one specified in the config file.
void parse_timestamp_offset_sampling(double cam1_to_cam2_timestamp_offset,
                                     std::string const& timestamp_offset_sampling,
                                     std::vector<double>& samples) {
  std::istringstream is(timestamp_offset_sampling);

  if (timestamp_offset_sampling == "") {
    samples.push_back(cam1_to_cam2_timestamp_offset);
  } else {
    std::string val;
    double beg, end;
    int num;

    if (!(is >> beg >> end >> num)) LOG(FATAL) << "Could not parse " << timestamp_offset_sampling << std::endl;

    if (beg > end || num <= 0) LOG(FATAL) << "The timestamp offset sampling parameters are invalid.";

    samples.clear();
    double delta = 0.0;
    if (num > 1) delta = (end - beg) / (num - 1);
    for (int it = 0; it < num - 1; it++) samples.push_back(beg + it * delta);
    samples.push_back(end);  // add the last one while avoiding any rounding error
  }
}

// Find the camera poses for the images having the calibration target
// (the calibration target is assumed to be fixed).
void find_world_to_cam_trans(  // Inputs
  std::string const& cam_name, boost::shared_ptr<camera::CameraParameters> cam_params,
  std::map<double, std::vector<std::vector<double>>> const& target_corners,
  // Outputs
  std::vector<double>& timestamps, std::vector<std::vector<Eigen::Vector2d>>& observations,
  std::vector<std::vector<Eigen::Vector3d>>& landmarks, std::vector<Eigen::Affine3d>& world_to_cam_vec) {
  // Wipe the inputs
  timestamps.clear();
  observations.clear();
  landmarks.clear();
  world_to_cam_vec.clear();

  for (auto it = target_corners.begin(); it != target_corners.end(); it++) {
    double timestamp = it->first;
    std::vector<std::vector<double>> const& corners = it->second;

    // Find the camera pose
    std::vector<Eigen::Vector2d> observations_per_image;
    std::vector<Eigen::Vector2d> undist_observations_per_image;
    std::vector<Eigen::Vector3d> landmarks_per_image;
    for (size_t pt_iter = 0; pt_iter < corners.size(); pt_iter++) {
      // The value  corners[pt_iter][0] is the corner id

      // Record the known measurements of the calibration target corners in meters
      landmarks_per_image.push_back(Eigen::Vector3d(corners[pt_iter][1], corners[pt_iter][2], 0.0));

      // Record the pixel position of the calibration corners as detected in the image
      Eigen::Vector2d target_corner_pixel;
      target_corner_pixel[0] = corners[pt_iter][3];  // projection into camera x in pixels
      target_corner_pixel[1] = corners[pt_iter][4];  // projection into camera y in pixels
      observations_per_image.push_back(target_corner_pixel);

      Eigen::Vector2d undist_observation;
      cam_params->Convert<camera::DISTORTED, camera::UNDISTORTED_C>(target_corner_pixel, &undist_observation);
      undist_observations_per_image.push_back(undist_observation);
    }

    Eigen::Affine3d world_to_cam;
    camera::CameraModel cam_model(world_to_cam, *cam_params.get());
    int ret = sparse_mapping::RansacEstimateCamera(landmarks_per_image, undist_observations_per_image,
                                                   FLAGS_calib_num_ransac_iterations,
                                                   FLAGS_calib_ransac_inlier_tolerance, &cam_model);
    if (ret != 0) continue;  // failed to estimate the camera

    world_to_cam = cam_model.GetTransform();
    timestamps.push_back(timestamp);
    observations.push_back(observations_per_image);
    landmarks.push_back(landmarks_per_image);
    world_to_cam_vec.push_back(world_to_cam);
  }
}

double calc_and_print_residual_stats(std::string const& prefix, std::string const& cam_name,
                                     std::vector<double> const& residuals) {
  std::vector<double> err_norm;
  size_t len = residuals.size() / 2;

  if (len == 0) {
    std::cout << "No residuals exist. This is likely an error.\n";
    return 0;
  }

  double res = 0.0;
  for (size_t it = 0; it < len; it++) {
    double norm = Eigen::Vector2d(residuals[2 * it + 0], residuals[2 * it + 1]).norm();
    res += norm;
    err_norm.push_back(norm);
  }
  res /= len;

  std::sort(err_norm.begin(), err_norm.end());
  std::cout << prefix << " min, mean, median and max reprojection error for "
            << cam_name << ": " << err_norm[0] << ' '
            << res << ' ' << err_norm[err_norm.size() / 2] << ' ' << err_norm.back() << std::endl;

  // Return the median error
  return err_norm[err_norm.size() / 2];
}

void save_residuals(std::string const& prefix, std::string const& cam_name,
                    std::string const& output_dir,
                    std::vector<Eigen::Vector2d> const& pixels,
                    std::vector<double> const& residuals) {
  size_t len = residuals.size() / 2;
  if (len != pixels.size()) {
    LOG(FATAL) << "Must have one residual norm per pixel.";
  }

  std::string res_file = output_dir + "/" + prefix + "-" + cam_name + "-residuals.txt";
  std::cout << "Writing residuals to: " << res_file << std::endl;
  std::ofstream ofs(res_file);
  ofs.precision(17);
  ofs << "# pixel_x, pixel_y, residual_x, residual_y, residual_norm\n";
  for (size_t it = 0; it < len; it++) {
    double norm = Eigen::Vector2d(residuals[2 * it + 0], residuals[2 * it + 1]).norm();
    ofs << pixels[it][0] << ", " << pixels[it][1] << ", " << residuals[2 * it + 0]
        << ", " << residuals[2 * it + 1] << ", " << norm << std::endl;
  }
  ofs.close();
}

// Refine a camera's intrinsics given a set images of the calibration target
// (refine along the way the position of the camera when those images were taken)
void refine_intrinsics(  // Inputs
  std::string const& cam_name, int num_focal_lengths, int num_iterations,
  std::set<std::string> const& cam_intrinsics_to_float, std::string const& output_dir,
  // Outputs
  boost::shared_ptr<camera::CameraParameters> cam_params, std::vector<std::vector<Eigen::Vector2d>>& observations,
  std::vector<std::vector<Eigen::Vector3d>>& landmarks, std::vector<Eigen::Affine3d>& world_to_cam_vec) {
  if (observations.size() != landmarks.size() || observations.size() != world_to_cam_vec.size())
    LOG(FATAL) << "Inconsistent input data for refining intrinsics.";

  // Set up the vector having the camera poses to optimzie
  int num_cams = world_to_cam_vec.size();
  std::vector<double> cameras(num_cams * NUM_RIGID_PARAMS);
  for (int it = 0; it < num_cams; it++) rigid_transform_to_array(world_to_cam_vec[it], &cameras[NUM_RIGID_PARAMS * it]);

  // Create storage space for the intrinsics
  Eigen::Vector2d focal_vector = cam_params->GetFocalVector();
  if (num_focal_lengths == 1) {
    focal_vector[0] = cam_params->GetFocalLength();  // average the two focal lengths
    focal_vector[1] = focal_vector[0];
  }
  std::cout << std::endl;
  std::cout << "Initial focal length for " << cam_name << ": " << focal_vector.transpose() << std::endl;
  Eigen::Vector2d optical_center = cam_params->GetOpticalOffset();
  std::cout << "Initial optical center for " << cam_name << ": " << optical_center.transpose() << std::endl;

  Eigen::VectorXd distortion = cam_params->GetDistortion();
  std::cout << "Initial distortion for " << cam_name << ": " << distortion.transpose() << std::endl;

  // Set up the variable blocks to optimize
  std::vector<int> block_sizes;
  block_sizes.push_back(num_focal_lengths);   // focal length
  block_sizes.push_back(NUM_OPT_CTR_PARAMS);  // optical center
  block_sizes.push_back(distortion.size());   // distortion
  block_sizes.push_back(NUM_RIGID_PARAMS);    // camera pose

  // build the problem
  ceres::Problem problem;
  std::vector<Eigen::Vector2d> pixels;
  for (int cam_it = 0; cam_it < num_cams; cam_it++) {
    for (size_t obs_it = 0; obs_it < observations[cam_it].size(); obs_it++) {
      pixels.push_back(observations[cam_it][obs_it]);
      ceres::CostFunction* cost_function =
        IntrinsicsError::Create(observations[cam_it][obs_it], landmarks[cam_it][obs_it], block_sizes, cam_params);

      ceres::LossFunction* loss_function = GetLossFunction("cauchy", FLAGS_robust_threshold);

      problem.AddResidualBlock(cost_function, loss_function, &focal_vector[0],
                               &optical_center[0], &distortion[0],
                               &cameras[NUM_RIGID_PARAMS * cam_it]);

      // In case it is ever desired to experiment with fixing the cameras
      // problem.SetParameterBlockConstant(&cameras[NUM_RIGID_PARAMS * cam_it]);

      // This logic must be in the loop, right after the residual got
      // added, otherwise one risks fixing parameters which were not
      // created yet.

      // See which values to float or keep fixed
      if (cam_intrinsics_to_float.find("focal_length") == cam_intrinsics_to_float.end()) {
        // std::cout << "For " << cam_name << " not floating focal_length." <<
        // std::endl;
        problem.SetParameterBlockConstant(&focal_vector[0]);
      } else {
        // std::cout << "For " << cam_name << " floating focal_length." <<
        // std::endl;
      }
      if (cam_intrinsics_to_float.find("optical_center") == cam_intrinsics_to_float.end()) {
        // std::cout << "For " << cam_name << " not floating optical_center." <<
        // std::endl;
        problem.SetParameterBlockConstant(&optical_center[0]);
      } else {
        // std::cout << "For " << cam_name << " floating optical_center." <<
        // std::endl;
      }
      if (cam_intrinsics_to_float.find("distortion") == cam_intrinsics_to_float.end()) {
        // std::cout << "For " << cam_name << " not floating distortion." <<
        // std::endl;
        problem.SetParameterBlockConstant(&distortion[0]);
      } else {
        // std::cout << "For " << cam_name << " floating distortion." << std::endl;
      }
    }
  }

  // Evaluate the residual before optimization
  double total_cost = 0.0;
  std::vector<double> err_norm;
  err_norm.clear();
  std::vector<double> residuals;
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.num_threads = 1;
  eval_options.apply_loss_function = false;  // want raw residuals
  problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);

  calc_and_print_residual_stats("Initial", cam_name, residuals);
  if (output_dir != "") save_residuals("initial", cam_name, output_dir, pixels, residuals);

  // Solve the problem
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = 1;  // The result is more predictable with one thread
  options.max_num_iterations = num_iterations;
  options.minimizer_progress_to_stdout = false;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.parameter_tolerance = FLAGS_parameter_tolerance;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << "\n";

  // Evaluate the residuals after optimization
  problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);

  calc_and_print_residual_stats("Final", cam_name, residuals);
  if (output_dir != "") save_residuals("final", cam_name, output_dir, pixels, residuals);

  if (num_focal_lengths == 1) focal_vector[1] = focal_vector[0];  // Only focal_vector[0] was used

  std::cout << "Final focal length for " << cam_name << ": " << focal_vector.transpose() << std::endl;
  std::cout << "Final optical center for " << cam_name << ": " << optical_center.transpose() << std::endl;
  std::cout << "Final distortion for " << cam_name << ": " << distortion.transpose() << std::endl;

  // Copy back the optimized intrinsics
  if (num_focal_lengths == 1)
    cam_params->SetFocalLength(Eigen::Vector2d(focal_vector[0], focal_vector[0]));
  else
    cam_params->SetFocalLength(focal_vector);
  cam_params->SetOpticalOffset(optical_center);
  cam_params->SetDistortion(distortion);

  // Copy back the optimized cameras
  for (int it = 0; it < num_cams; it++) array_to_rigid_transform(world_to_cam_vec[it], &cameras[NUM_RIGID_PARAMS * it]);
}

// Find the depth cam intrinsics so that the target points coordinates as measured
// by the depth camera project onto the target corners as seen in the intensity image.
void refine_depth_cam_intrinsics(  // Inputs
  std::string const& cam_name, int num_focal_lengths, int num_iterations,
  std::set<std::string> const& cam_intrinsics_to_float,
  std::map<double, std::vector<std::vector<double>>> const& cam_target_corners,
  std::map<double, std::vector<Eigen::Vector3d>> const& target_corners_depth,
  // Outputs
  boost::shared_ptr<camera::CameraParameters> depth_cam_params) {
  // Create storage space for the intrinsics
  Eigen::Vector2d focal_vector = depth_cam_params->GetFocalVector();
  if (num_focal_lengths == 1) {
    focal_vector[0] = depth_cam_params->GetFocalLength();  // average the two focal lengths
    focal_vector[1] = focal_vector[0];
  }
  std::cout << std::endl;
  std::cout << "Initial focal length for " << cam_name << ": " << focal_vector.transpose() << std::endl;
  Eigen::Vector2d optical_center = depth_cam_params->GetOpticalOffset();
  std::cout << "Initial optical center for " << cam_name << ": " << optical_center.transpose() << std::endl;
  Eigen::VectorXd distortion = depth_cam_params->GetDistortion();
  std::cout << "Initial distortion for " << cam_name << ": " << distortion.transpose() << std::endl;

  std::vector<int> block_sizes;
  block_sizes.push_back(num_focal_lengths);   // focal length
  block_sizes.push_back(NUM_OPT_CTR_PARAMS);  // optical center
  block_sizes.push_back(distortion.size());   // distortion

  // build the problem
  ceres::Problem problem;
  for (auto it = cam_target_corners.begin(); it != cam_target_corners.end(); it++) {
    // This has the target pixel corners
    std::vector<std::vector<double>> const& target_corners = it->second;

    // Look up corresponding depth measurements
    double timestamp = it->first;
    auto depth_it = target_corners_depth.find(timestamp);
    if (depth_it == target_corners_depth.end()) continue;
    std::vector<Eigen::Vector3d> const& depth_vec = depth_it->second;

    if (target_corners.size() != depth_vec.size())
      LOG(FATAL) << "Must have one depth measurement for each target corner.";

    // Add a cost function residual for each target corner
    for (size_t pt_iter = 0; pt_iter < target_corners.size(); pt_iter++) {
      Eigen::Vector2d target_corner_pixel;
      // The value  target_corners[pt_iter][0] is the corner id
      // Then come the world measurements, and pixel measurements
      target_corner_pixel[0] = target_corners[pt_iter][3];  // pixel x
      target_corner_pixel[1] = target_corners[pt_iter][4];  // pixel y

      Eigen::Vector3d depth = depth_vec[pt_iter];

      // Ignore invalid depth values
      if (depth == Eigen::Vector3d(0.0, 0.0, 0.0)) continue;

      ceres::CostFunction* cost_function =
        IntrinsicsDepthError::Create(target_corner_pixel, depth, block_sizes, depth_cam_params);

      ceres::LossFunction* loss_function = GetLossFunction("cauchy", FLAGS_robust_threshold);

      problem.AddResidualBlock(cost_function, loss_function, &focal_vector[0], &optical_center[0], &distortion[0]);
    }
  }

  // See which values to float or keep fixed
  if (cam_intrinsics_to_float.find("focal_length") == cam_intrinsics_to_float.end()) {
    // std::cout << "For " << cam_name << " not floating focal_length." <<
    // std::endl;
    problem.SetParameterBlockConstant(&focal_vector[0]);
  } else {
    // std::cout << "For " << cam_name << " floating focal_length." <<
    // std::endl;
  }
  if (cam_intrinsics_to_float.find("optical_center") == cam_intrinsics_to_float.end()) {
    // std::cout << "For " << cam_name << " not floating optical_center." <<
    // std::endl;
    problem.SetParameterBlockConstant(&optical_center[0]);
  } else {
    // std::cout << "For " << cam_name << " floating optical_center." <<
    // std::endl;
  }
  if (cam_intrinsics_to_float.find("distortion") == cam_intrinsics_to_float.end()) {
    // std::cout << "For " << cam_name << " not floating distortion." <<
    // std::endl;
    problem.SetParameterBlockConstant(&distortion[0]);
  } else {
    // std::cout << "For " << cam_name << " floating distortion." << std::endl;
  }

  // Evaluate the residual before optimization
  double total_cost = 0.0;
  std::vector<double> residuals;
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.num_threads = 1;
  eval_options.apply_loss_function = false;  // want raw residuals
  problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);
  calc_and_print_residual_stats("Initial", cam_name, residuals);

  // Solve the problem
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = 1;  // The result is more predictable with one thread
  options.max_num_iterations = num_iterations;
  options.minimizer_progress_to_stdout = false;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.parameter_tolerance = FLAGS_parameter_tolerance;
  ceres::Solve(options, &problem, &summary);

  // Evaluate the residuals after optimization
  problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);
  calc_and_print_residual_stats("Final", cam_name, residuals);

  if (num_focal_lengths == 1) focal_vector[1] = focal_vector[0];  // Only focal_vector[0] was used

  std::cout << "Final focal length for " << cam_name << ": " << focal_vector.transpose() << std::endl;
  std::cout << "Final optical center for " << cam_name << ": " << optical_center.transpose() << std::endl;
  std::cout << "Final distortion for " << cam_name << ": " << distortion.transpose() << std::endl;

  // Copy back the optimized intrinsics
  if (num_focal_lengths == 1)
    depth_cam_params->SetFocalLength(Eigen::Vector2d(focal_vector[0], focal_vector[0]));
  else
    depth_cam_params->SetFocalLength(focal_vector);
  depth_cam_params->SetOpticalOffset(optical_center);
  depth_cam_params->SetDistortion(distortion);
}

// Find the matrix by which to multiply the measured depth clouds to correct for
// their incorrect scale.
void find_depth_to_image_transform(std::vector<double> const& cam2_timestamps,
                                   std::vector<Eigen::Affine3d> const& world_to_cam2_vec,
                                   std::map<double, std::vector<std::vector<double>>> const& cam_target_corners,
                                   std::map<double, std::vector<Eigen::Vector3d>> const& target_corners_depth,
                                   std::string const& cam2_name,
                                   boost::shared_ptr<camera::CameraParameters> depth_cam_params,
                                   Eigen::Affine3d& depth_to_image_transform) {
  if (cam2_timestamps.size() != world_to_cam2_vec.size()) LOG(FATAL) << "Expecting as many timestamps as cameras.";

  // Store in a simple vector structure the valid depth values and
  // points on the calibration target transformed to the depth camera
  // coordinate system.
  std::vector<Eigen::Vector3d> depth_vec, trans_target_vec;

  // Iterate over images
  for (auto it = cam_target_corners.begin(); it != cam_target_corners.end(); it++) {
    // An alias for the target corners for the given image
    std::vector<std::vector<double>> const& target_corners = it->second;

    // Look up corresponding depth measurements
    double timestamp = it->first;
    auto depth_it = target_corners_depth.find(timestamp);
    if (depth_it == target_corners_depth.end()) continue;
    std::vector<Eigen::Vector3d> const& depth_per_image_vec = depth_it->second;

    if (target_corners.size() != depth_per_image_vec.size())
      LOG(FATAL) << "Must have one depth measurement for each target corner.";

    // Look up the camera transform for this timestamp.  This is an
    // O(N) operation, which makes the whole algorithm O(N^2). Luckily
    // N is small.
    auto cam_index = std::find(cam2_timestamps.begin(), cam2_timestamps.end(), timestamp);
    if (cam_index == cam2_timestamps.end()) continue;
    Eigen::Affine3d world_to_cam = world_to_cam2_vec[cam_index - cam2_timestamps.begin()];

    // Look at each target corner
    for (size_t pt_iter = 0; pt_iter < target_corners.size(); pt_iter++) {
      Eigen::Vector3d target_pt;
      Eigen::Vector2d target_corner_pixel;

      // The value  target_corners[pt_iter][0] is the corner id.
      // Then come the world measurements, and pixel measurements.
      target_pt[0] = target_corners[pt_iter][1];
      target_pt[1] = target_corners[pt_iter][2];
      target_pt[2] = 0.0;
      target_corner_pixel[0] = target_corners[pt_iter][3];
      target_corner_pixel[1] = target_corners[pt_iter][4];

      Eigen::Vector3d depth_pt = depth_per_image_vec[pt_iter];

      // Ignore invalid depth values
      if (depth_pt == Eigen::Vector3d(0.0, 0.0, 0.0)) continue;

      // Save the current depth and target corner measurement
      // transformed to current camera's coordinates.
      Eigen::Vector3d trans_target_pt = world_to_cam * target_pt;

      // Normally by now the depth camera is well-calibrated, so we
      // are correcting for a small scale difference of a few percent.
      // So the discrepancy between these two different way of
      // measuring a point should not be that big, at most 0.1 m.
      if ((depth_pt - trans_target_pt).norm() > 0.4) {
        std::cout << "Found a large discrepancy between a target point and its "
                  << "depth measurement. Will be skipped. Their values are: " << trans_target_pt.transpose() << " and "
                  << depth_pt.transpose() << "\n";
        continue;
      }

      depth_vec.push_back(depth_pt);
      trans_target_vec.push_back(trans_target_pt);
    }
  }

  size_t np = trans_target_vec.size();
  Eigen::Matrix3Xd in(3, np), out(3, np);
  for (size_t i = 0; i < np; i++) {
    in.col(i) = depth_vec[i];
    out.col(i) = trans_target_vec[i];
  }
  sparse_mapping::Find3DAffineTransform(in, out, &depth_to_image_transform);

  double trans_res = 0.0;
  int num = 0;
  std::vector<double> err_norm;
  for (size_t i = 0; i < np; i++) {
    Eigen::Vector3d dP = depth_vec[i];
    Eigen::Vector3d iP = trans_target_vec[i];
    Eigen::Vector3d rs = depth_to_image_transform * dP - iP;
    double norm = rs.norm();
    err_norm.push_back(norm);
    trans_res += norm;
    num += 1;
  }
  trans_res /= num;

  std::sort(err_norm.begin(), err_norm.end());
  std::cout << std::endl;
  std::cout << "Min, mean, median max depth to image transform error (meters): " << err_norm[0] << ' ' << trans_res
            << ' ' << err_norm[err_norm.size() / 2] << ' ' << err_norm.back() << std::endl;
  std::cout << "Depth to image transform for " << FLAGS_cam2_name << ":\n"
            << depth_to_image_transform.matrix() << std::endl;
}

// Based on cam1 and cam2 poses find the initial transform between them
// by averaging a set of transforms
void find_initial_extrinsics(std::vector<double> const& cam1_timestamps, std::vector<double> const& cam2_timestamps,
                             std::vector<Eigen::Affine3d> const& world_to_cam1_vec,
                             std::vector<Eigen::Affine3d> const& world_to_cam2_vec,
                             double cam1_to_cam2_timestamp_offset, double max_interp_dist,
                             Eigen::Affine3d& cam2_to_cam1_transform) {
  // Keep track of the index of the cam1 timestamp right before
  // the current cam2 timestamp. It will increase as time goes
  // by. This will help quickly look up cam1 times.
  int left_cam1_cid = 0;
  int num = 0;

  // Eigen does not initialize these automatically to zero.
  Eigen::Quaternion<double> avg_q(0, 0, 0, 0);
  Eigen::Vector3d avg_t(0, 0, 0);

  for (size_t cam2_cid = 0; cam2_cid < cam2_timestamps.size(); cam2_cid++) {
    double cam2_time = cam2_timestamps[cam2_cid];
    Eigen::Affine3d world_to_cam2 = world_to_cam2_vec[cam2_cid];

    // Cast to int below, to avoid issues with subtracting 1 from size_t.
    int num_timestamps = cam1_timestamps.size();
    for (int cam1_cid = left_cam1_cid; cam1_cid < num_timestamps - 1; cam1_cid++) {
      double left_cam1_time = cam1_timestamps[cam1_cid + 0] + cam1_to_cam2_timestamp_offset;
      double right_cam1_time = cam1_timestamps[cam1_cid + 1] + cam1_to_cam2_timestamp_offset;

      // Find the cam1 times bracketing the current cam2 time
      if (right_cam1_time < cam2_time) {
        // Too early
        continue;
      }
      if (left_cam1_time >= cam2_time) {
        // Too late
        break;
      }

      // For the next cam2_time will start the search at this index
      left_cam1_cid = cam1_cid;

      if (std::abs(left_cam1_time - right_cam1_time) > max_interp_dist) continue;

      double alpha = (cam2_time - left_cam1_time) / (right_cam1_time - left_cam1_time);

      Eigen::Affine3d beg_world_to_cam1 = world_to_cam1_vec[cam1_cid + 0];
      Eigen::Affine3d end_world_to_cam1 = world_to_cam1_vec[cam1_cid + 1];

      Eigen::Affine3d interp_world_to_cam1 = dense_map::linearInterp(alpha, beg_world_to_cam1, end_world_to_cam1);
      Eigen::Affine3d T = interp_world_to_cam1 * world_to_cam2.inverse();
      Eigen::Quaternion<double> q(T.linear());
      //  std::cout << "q " << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << std::endl;
      //  std::cout << "t " << T.translation().transpose() << std::endl;

      // There is no operation to add quaternions
      avg_q.x() += q.x();
      avg_q.y() += q.y();
      avg_q.z() += q.z();
      avg_q.w() += q.w();
      avg_t += T.translation();
      num++;
    }
  }

  if (num == 0) LOG(FATAL) << "Could not find an initial guess for extrinsics. Maybe more data is needed.";

  avg_q.x() /= num;
  avg_q.y() /= num;
  avg_q.z() /= num;
  avg_q.w() /= num;
  avg_t /= num;
  avg_q.normalize();

  // std::cout << "avg q " << avg_q.x() << ' ' << avg_q.y() << ' ' << avg_q.z() << ' ' << avg_q.w()
  //            << std::endl;
  // std::cout << "avg t " << avg_t.transpose() << std::endl;

  cam2_to_cam1_transform =
    Eigen::Affine3d(Eigen::Translation3d(avg_t.x(), avg_t.y(), avg_t.z())) * Eigen::Affine3d(avg_q);
}

// Refine cam2_to_cam1_transform
void refine_extrinsics(int num_iterations, std::vector<double> const& cam1_timestamps,
                       std::vector<double> const& cam2_timestamps,
                       std::vector<Eigen::Affine3d> const& world_to_cam1_vec,
                       double cam1_to_cam2_timestamp_offset,
                       double max_interp_dist,
                       boost::shared_ptr<camera::CameraParameters> const& cam2_params,
                       std::map<double, std::vector<std::vector<double>>> const& cam2_target_corners,
                       Eigen::Affine3d& cam2_to_cam1_transform, double& median_error) {
  // Convert the initial transform to vector format, to pass to the solver
  std::vector<double> cam2_to_cam1_transform_vec(NUM_RIGID_PARAMS);
  rigid_transform_to_array(cam2_to_cam1_transform, &cam2_to_cam1_transform_vec[0]);

  std::vector<int> block_sizes;
  block_sizes.push_back(NUM_RIGID_PARAMS);  // block for cam2_to_cam1_transform_vec

  // build the problem
  ceres::Problem problem;
  ceres::LossFunction* loss_function = GetLossFunction("cauchy", FLAGS_robust_threshold);

  // Keep track of the index of the cam1 timestamp right before
  // the current cam2 timestamp. It will increase as time goes
  // by. This will help quickly look up cam1 times.
  int left_cam1_cid = 0;

  for (size_t cam2_cid = 0; cam2_cid < cam2_timestamps.size(); cam2_cid++) {
    double cam2_time = cam2_timestamps[cam2_cid];
    //  Eigen::Affine3d world_to_cam2 = cam2_map.cid_to_cam_t_global_[cam2_cid];

    int num_cam1_times = cam1_timestamps.size();  // cast to int to avoid issues below
    for (int cam1_cid = left_cam1_cid; cam1_cid < num_cam1_times - 1; cam1_cid++) {
      double left_cam1_time = cam1_timestamps[cam1_cid + 0] + cam1_to_cam2_timestamp_offset;
      double right_cam1_time = cam1_timestamps[cam1_cid + 1] + cam1_to_cam2_timestamp_offset;

      // Find the cam1 times bracketing the current cam2 time
      if (right_cam1_time < cam2_time) {
        // Too early
        continue;
      }
      if (left_cam1_time >= cam2_time) {
        // Too late
        break;
      }

      // For the next cam2_time will start the search at this index
      left_cam1_cid = cam1_cid;

      if (std::abs(left_cam1_time - right_cam1_time) > max_interp_dist) {
        // std::cout << "--timestamp with large difference, will skip: "
        //           << left_cam1_time << ' ' << right_cam1_time << ' '
        //           << right_cam1_time - left_cam1_time << std::endl;
        continue;
      }

      Eigen::Affine3d left_world_to_cam1 = world_to_cam1_vec[cam1_cid + 0];
      Eigen::Affine3d right_world_to_cam1 = world_to_cam1_vec[cam1_cid + 1];

      double alpha = (cam2_time - left_cam1_time) / (right_cam1_time - left_cam1_time);

      Eigen::Affine3d interp_world_to_cam1
        = dense_map::linearInterp(alpha, left_world_to_cam1, right_world_to_cam1);

      auto it = cam2_target_corners.find(cam2_time);
      if (it == cam2_target_corners.end()) continue;
      std::vector<std::vector<double>> const& target_corners = it->second;

      for (size_t pt_iter = 0; pt_iter < target_corners.size(); pt_iter++) {
        Eigen::Vector2d target_corner_meas, target_corner_pixel;
        // The value  target_corners[pt_iter][0] is the corner id
        target_corner_meas[0] = target_corners[pt_iter][1];   // real world measurement x
        target_corner_meas[1] = target_corners[pt_iter][2];   // real world measurement y
        target_corner_pixel[0] = target_corners[pt_iter][3];  // projection into camera x
        target_corner_pixel[1] = target_corners[pt_iter][4];  // projection into camera y

        ceres::CostFunction* cost_function
          = ExtrinsicsError::Create(target_corner_pixel, target_corner_meas,
                                    interp_world_to_cam1, block_sizes, cam2_params);
        problem.AddResidualBlock(cost_function, loss_function, &cam2_to_cam1_transform_vec[0]);
      }
    }
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = 1;  // The result is more predictable with one thread
  options.max_num_iterations = num_iterations;
  options.minimizer_progress_to_stdout = false;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.parameter_tolerance = FLAGS_parameter_tolerance;

  // Find the initial residuals
  double total_cost = 0.0;
  std::vector<double> residuals;
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.num_threads = 1;
  eval_options.apply_loss_function = false;  // want raw residuals
  problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);
  calc_and_print_residual_stats("Initial", "extrinsics", residuals);

  // Optimize the transform
  ceres::Solve(options, &problem, &summary);

  // Find the final residuals. Save the median error.
  problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);
  median_error = calc_and_print_residual_stats("Final", "extrinsics", residuals);

  // Update cam2_to_cam1_transform
  array_to_rigid_transform(cam2_to_cam1_transform, &cam2_to_cam1_transform_vec[0]);
}
}  // namespace dense_map

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  if (FLAGS_cam1_dir.empty() || FLAGS_cam2_dir.empty())
    LOG(FATAL) << "Not all inputs were specified.";

  if ((FLAGS_cam1_name != "nav_cam" && FLAGS_cam1_name != "sci_cam") ||
      FLAGS_cam2_name != "haz_cam")
    LOG(FATAL) << "Must specify -cam1_name as one of nav_cam or sci_cam,"
               << " and -cam2_name as haz_cam.";

  // Read the calibration target pixel corners and their known 3D locations
  std::map<double, std::vector<std::vector<double>>> cam1_target_corners;
  dense_map::read_target_corners(  // Inputs
    FLAGS_start, FLAGS_duration, FLAGS_cam1_dir,
    // Outputs
    cam1_target_corners);

  std::map<double, std::vector<std::vector<double>>> cam2_target_corners;
  dense_map::read_target_corners(  // Inputs
    FLAGS_start, FLAGS_duration, FLAGS_cam2_dir,
    // Outputs
    cam2_target_corners);

  // For the depth cam, read the depth measurements at the target points.
  std::map<double, std::vector<Eigen::Vector3d>> target_corners_depth;
  dense_map::read_depth_cam_meas(  // Inputs
    FLAGS_bag, FLAGS_haz_cam_points_topic, cam2_target_corners,
    // Outputs
    target_corners_depth);

  std::set<std::string> cam1_intrinsics_to_float, cam2_intrinsics_to_float;
  dense_map::parse_intrinsics_to_float(FLAGS_cam1_intrinsics_to_float, cam1_intrinsics_to_float);
  dense_map::parse_intrinsics_to_float(FLAGS_cam2_intrinsics_to_float, cam2_intrinsics_to_float);

  // Read the bot config file
  std::vector<std::string> cam_names = {"nav_cam", "haz_cam", "sci_cam"};
  std::vector<camera::CameraParameters> cam_params;
  std::vector<Eigen::Affine3d>          ref_to_cam_trans;
  std::vector<double>                   ref_to_cam_timestamp_offsets;
  Eigen::Affine3d                       navcam_to_body_trans;
  Eigen::Affine3d                       hazcam_depth_to_image_transform;
  dense_map::readConfigFile(  // Inputs
    cam_names, "nav_cam_transform", "haz_cam_depth_to_image_transform",
    // Outputs
    cam_params, ref_to_cam_trans, ref_to_cam_timestamp_offsets, navcam_to_body_trans,
    hazcam_depth_to_image_transform);

  Eigen::Affine3d hazcam_to_navcam_aff_trans = ref_to_cam_trans[1].inverse();
  Eigen::Affine3d scicam_to_hazcam_aff_trans =
    ref_to_cam_trans[1] * ref_to_cam_trans[2].inverse();
  double navcam_to_hazcam_timestamp_offset = ref_to_cam_timestamp_offsets[1];
  double scicam_to_hazcam_timestamp_offset =
    ref_to_cam_timestamp_offsets[1] - ref_to_cam_timestamp_offsets[2];

  // read cam1 params
  boost::shared_ptr<camera::CameraParameters> cam1_params;
  boost::shared_ptr<camera::CameraParameters> cam2_params;
  double cam1_to_cam2_timestamp_offset = 0.0;
  std::string cam1_to_cam2_timestamp_offset_str;
  Eigen::Affine3d cam2_to_cam1_transform;
  if (FLAGS_cam1_name == "nav_cam" && FLAGS_cam2_name == "haz_cam") {
    cam1_params = boost::shared_ptr<camera::CameraParameters>
      (new camera::CameraParameters(cam_params[0]));
    cam2_params = boost::shared_ptr<camera::CameraParameters>
      (new camera::CameraParameters(cam_params[1]));
    cam1_to_cam2_timestamp_offset     = navcam_to_hazcam_timestamp_offset;
    cam1_to_cam2_timestamp_offset_str = "navcam_to_hazcam_timestamp_offset";
    cam2_to_cam1_transform = hazcam_to_navcam_aff_trans;
  } else if (FLAGS_cam1_name == "sci_cam" && FLAGS_cam2_name == "haz_cam") {
    cam1_params = boost::shared_ptr<camera::CameraParameters>
      (new camera::CameraParameters(cam_params[2]));
    cam2_params = boost::shared_ptr<camera::CameraParameters>
      (new camera::CameraParameters(cam_params[1]));
    cam1_to_cam2_timestamp_offset = scicam_to_hazcam_timestamp_offset;
    cam1_to_cam2_timestamp_offset_str = "scicam_to_hazcam_timestamp_offset";
    cam2_to_cam1_transform = scicam_to_hazcam_aff_trans.inverse();
  } else {
    LOG(FATAL) << "Must specify -cam1_name as nav_cam or sci_cam, "
               << "and -cam2_name as haz_cam.";
  }

  if (FLAGS_num_cam1_focal_lengths != 1 && FLAGS_num_cam1_focal_lengths != 2)
    LOG(FATAL) << "Can use 1 or 2 focal lengths.";
  if (FLAGS_num_cam2_focal_lengths != 1 && FLAGS_num_cam2_focal_lengths != 2)
    LOG(FATAL) << "Can use 1 or 2 focal lengths.";

  if (FLAGS_output_dir != "") {
    if (!boost::filesystem::exists(FLAGS_output_dir)) {
      int status = mkdir(FLAGS_output_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      if (status && errno != EEXIST) {
        LOG(FATAL) << "Failed to create directory: " << FLAGS_output_dir << ".";
        return 1;
      }
    }
  }

  std::vector<double> timestamp_offset_samples;
  dense_map::parse_timestamp_offset_sampling(cam1_to_cam2_timestamp_offset,
                                             FLAGS_timestamp_offset_sampling,
                                             timestamp_offset_samples);

  // Find cam1 poses and timestamps
  std::vector<double> cam1_timestamps;
  std::vector<std::vector<Eigen::Vector2d>> cam1_observations;
  std::vector<std::vector<Eigen::Vector3d>> cam1_landmarks;
  std::vector<Eigen::Affine3d> world_to_cam1_vec;
  dense_map::find_world_to_cam_trans(FLAGS_cam1_name, cam1_params, cam1_target_corners,
                                     // Outputs
                                     cam1_timestamps, cam1_observations,
                                     cam1_landmarks, world_to_cam1_vec);

  // Refine cam1_params and world_to_cam1_vec
  if (FLAGS_update_cam1)
    dense_map::refine_intrinsics(  // Inputs
      FLAGS_cam1_name, FLAGS_num_cam1_focal_lengths, FLAGS_num_cam1_iterations,
      cam1_intrinsics_to_float, FLAGS_output_dir,
      // Outputs
      cam1_params, cam1_observations, cam1_landmarks, world_to_cam1_vec);
  // Refine cam2_params. It is very important to note that here we
  // want the 3D points as measured by the depth camera to project
  // accurately to the target corner pixels, unlike for the intrinsics
  // of cam1, where we wanted the measured 3D target corners to
  // project well to the target corner pixels. That because cam2 is a
  // depth camera while cam1 is not.
  if (FLAGS_update_cam2)
    dense_map::refine_depth_cam_intrinsics(  // Inputs
      FLAGS_cam2_name, FLAGS_num_cam2_focal_lengths, FLAGS_num_cam2_iterations,
      cam2_intrinsics_to_float, cam2_target_corners, target_corners_depth,
      // Outputs
      cam2_params);

  // Find cam2 poses and timestamps
  std::vector<double> cam2_timestamps;
  std::vector<std::vector<Eigen::Vector2d>> cam2_observations;
  std::vector<std::vector<Eigen::Vector3d>> cam2_landmarks;
  std::vector<Eigen::Affine3d> world_to_cam2_vec;
  dense_map::find_world_to_cam_trans(FLAGS_cam2_name, cam2_params, cam2_target_corners,
                                     cam2_timestamps, cam2_observations, cam2_landmarks,
                                     world_to_cam2_vec);

  // Find the transform to correct the scale of the haz cam depth clouds
  if (FLAGS_update_depth_to_image_transform)
    dense_map::find_depth_to_image_transform(  // Inputs
      cam2_timestamps, world_to_cam2_vec, cam2_target_corners, target_corners_depth,
      FLAGS_cam2_name, cam2_params,
      // Outputs
      hazcam_depth_to_image_transform);

  // Find the transform from cam2 to cam1.
  if (FLAGS_update_extrinsics) {
    // If provided, try several values
    // for cam1_to_cam2_timestamp_offset and pick the one with the smallest
    // median error.
    if (FLAGS_timestamp_offset_sampling != "") {
      std::cout << std::endl;
      std::cout << "Sampling the timestamp offset from " << timestamp_offset_samples.front()
                << " to " << timestamp_offset_samples.back() << " with "
                << timestamp_offset_samples.size() << " sample(s)." << std::endl;
    }

    double median_extrinsics_err = std::numeric_limits<double>::max();
    for (size_t it = 0; it < timestamp_offset_samples.size(); it++) {
      double curr_cam1_to_cam2_timestamp_offset = timestamp_offset_samples[it];
      double curr_median_extrinsics_err;
      Eigen::Affine3d curr_cam2_to_cam1_transform;

      std::cout << std::endl;
      std::cout << "Finding extrinsics with " << cam1_to_cam2_timestamp_offset_str << " "
                << curr_cam1_to_cam2_timestamp_offset << std::endl;

      dense_map::find_initial_extrinsics(cam1_timestamps, cam2_timestamps,
                                         world_to_cam1_vec, world_to_cam2_vec,
                                         curr_cam1_to_cam2_timestamp_offset,
                                         FLAGS_max_interp_dist,
                                         curr_cam2_to_cam1_transform);
      dense_map::refine_extrinsics(FLAGS_num_extrinsics_iterations, cam1_timestamps,
                                   cam2_timestamps, world_to_cam1_vec,
                                   curr_cam1_to_cam2_timestamp_offset, FLAGS_max_interp_dist,
                                   cam2_params,
                                   cam2_target_corners, curr_cam2_to_cam1_transform,
                                   curr_median_extrinsics_err);
      if (curr_median_extrinsics_err < median_extrinsics_err) {
        cam1_to_cam2_timestamp_offset = curr_cam1_to_cam2_timestamp_offset;
        cam2_to_cam1_transform = curr_cam2_to_cam1_transform;
        median_extrinsics_err = curr_median_extrinsics_err;
      }
    }

    std::cout << std::endl;
    std::cout << "Final value of " << cam1_to_cam2_timestamp_offset_str << " is "
              << cam1_to_cam2_timestamp_offset
              << " with smallest median error of " << median_extrinsics_err
              << " pixels" << std::endl;

    std::cout << std::endl;
    std::cout << "Extrinsics transform from " << FLAGS_cam2_name << " to "
              << FLAGS_cam1_name << ":\n"
              << cam2_to_cam1_transform.matrix() << std::endl;
  }

  // Save back the results
  if (FLAGS_cam1_name == "nav_cam" && FLAGS_cam2_name == "haz_cam") {
    cam_params[0] = *cam1_params;
    cam_params[1] = *cam2_params;
    navcam_to_hazcam_timestamp_offset = cam1_to_cam2_timestamp_offset;
    hazcam_to_navcam_aff_trans = cam2_to_cam1_transform;
  } else if (FLAGS_cam1_name == "sci_cam" && FLAGS_cam2_name == "haz_cam") {
    cam_params[2] = *cam1_params;
    cam_params[1] = *cam2_params;
    scicam_to_hazcam_timestamp_offset = cam1_to_cam2_timestamp_offset;
    scicam_to_hazcam_aff_trans = cam2_to_cam1_transform.inverse();
  } else {
    LOG(FATAL) << "Must specify -cam1_name as nav_cam or sci_cam, "
               << "and -cam2_name as haz_cam.";
  }

  // Recall that cam_names = {"nav_cam", "haz_cam", "sci_cam"};
  // nav_to_haz
  ref_to_cam_trans[1] = hazcam_to_navcam_aff_trans.inverse();
  // nav_to_sci
  ref_to_cam_trans[2] = scicam_to_hazcam_aff_trans.inverse() *
    hazcam_to_navcam_aff_trans.inverse();
  ref_to_cam_timestamp_offsets[1] = navcam_to_hazcam_timestamp_offset;
  ref_to_cam_timestamp_offsets[2] =
    navcam_to_hazcam_timestamp_offset - scicam_to_hazcam_timestamp_offset;
  dense_map::updateConfigFile(cam_names, "haz_cam_depth_to_image_transform",
                              cam_params, ref_to_cam_trans,
                              ref_to_cam_timestamp_offsets,
                              hazcam_depth_to_image_transform);

  return 0;
}
