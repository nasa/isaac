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

// TODO(oalexan1): Consider adding a haz cam to haz cam
// reprojection error in the camera refiner. There will be more
// haz to haz matches than haz to nav or haz to sci.
// TODO(oalexan1): Must document that the cloud timestamp
// that is looked up is what is closest to image timestamp.
// TODO(oalexan1): What if the wrong cloud is looked up
// for given image? Or if the delay is too big?

// TODO(oalexan1): Must test the DepthError with no bracketing.

// TODO(oalexan1): Move this to utils
// Get rid of warning beyond our control
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic push
#include <openMVG/multiview/projection.hpp>
#include <openMVG/multiview/rotation_averaging_l1.hpp>
#include <openMVG/multiview/triangulation_nview.hpp>
#include <openMVG/numeric/numeric.h>
#include <openMVG/tracks/tracks.hpp>
#pragma GCC diagnostic pop

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/dynamic_numeric_diff_cost_function.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/autodiff_cost_function.h>

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
#include <sparse_mapping/tensor.h>
#include <config_reader/config_reader.h>
#include <msg_conversions/msg_conversions.h>

#include <dense_map_ros_utils.h>
#include <dense_map_utils.h>
#include <interest_point.h>
#include <texture_processing.h>

#include <string>
#include <map>
#include <array>
#include <iostream>
#include <fstream>

DEFINE_string(ros_bag, "", "A ROS bag with recorded nav_cam, haz_cam, and full-resolution sci_cam data.");

DEFINE_string(sparse_map, "",
              "A registered SURF sparse map made with some of the ROS bag data, "
              "and including nav cam images closely bracketing the sci cam images.");

DEFINE_string(output_map, "", "Output file containing the updated map.");

DEFINE_string(nav_cam_topic, "/hw/cam_nav", "The nav cam topic in the bag file.");

DEFINE_string(haz_cam_points_topic, "/hw/depth_haz/points", "The depth point cloud topic in the bag file.");

DEFINE_string(haz_cam_intensity_topic, "/hw/depth_haz/extended/amplitude_int",
              "The depth camera intensity topic in the bag file.");

DEFINE_string(sci_cam_topic, "/hw/cam_sci/compressed", "The sci cam topic in the bag file.");

DEFINE_double(start, 0.0, "How many seconds into the bag to start processing the data.");

DEFINE_double(duration, -1.0, "For how many seconds to do the processing.");

DEFINE_double(max_haz_cam_image_to_depth_timestamp_diff, 0.2,
              "Use depth haz cam clouds that are within this distance in "
              "time from the nearest haz cam intensity image.");

DEFINE_double(robust_threshold, 3.0,
              "Pixel errors much larger than this will be exponentially attenuated "
              "to affect less the cost function.");

DEFINE_int32(num_iterations, 20, "How many solver iterations to perform in calibration.");

DEFINE_double(parameter_tolerance, 1e-12, "Stop when the optimization variables change by less than this.");

DEFINE_double(bracket_len, 2.0,
              "Lookup sci and haz cam images only between consecutive nav cam images "
              "whose distance in time is no more than this (in seconds), after adjusting "
              "for the timestamp offset between these cameras. It is assumed the robot "
              "moves slowly and uniformly during this time.");

DEFINE_int32(num_opt_threads, 16, "How many threads to use in the optimization.");

DEFINE_string(hugin_file, "", "The path to the hugin .pto file used for sparse map registration.");

DEFINE_string(xyz_file, "", "The path to the xyz file used for sparse map registration.");

DEFINE_bool(timestamp_interpolation, false,
            "If true, interpolate between "
            "timestamps. May give better results if the robot is known to move "
            "uniformly, and perhaps less so for stop-and-go robot motion.");

DEFINE_string(sci_cam_timestamps, "",
              "Use only these sci cam timestamps. Must be "
              "a file with one timestamp per line.");

DEFINE_bool(skip_registration, false,
            "If true, do not re-register the optimized map. "
            "Then the hugin and xyz file options need not be provided. "
            "This may result in the scale not being correct if the sparse map is not fixed.");

DEFINE_bool(opt_map_only, false, "If to optimize only the map and not the camera params.");

DEFINE_bool(fix_map, false, "Do not optimize the sparse map, hence only the camera params.");

DEFINE_bool(float_scale, false,
            "If to optimize the scale of the clouds (use it if the "
            "sparse map is kept fixed).");

DEFINE_string(sci_cam_intrinsics_to_float, "",
              "Refine 0 or more of the following intrinsics for sci_cam: focal_length, "
              "optical_center, distortion. Specify as a quoted list. "
              "For example: 'focal_length optical_center'.");

DEFINE_double(scicam_to_hazcam_timestamp_offset_override_value,
              std::numeric_limits<double>::quiet_NaN(),
              "Override the value of scicam_to_hazcam_timestamp_offset from the robot config "
              "file with this value.");

DEFINE_double(depth_weight, 1000.0,
              "The weight to give to depth measurements.");

DEFINE_string(mesh, "",
              "Refine the sci cam so that the sci cam texture agrees with the nav cam "
              "texture when projected on this mesh.");

DEFINE_double(mesh_weight, 25.0,
              "A larger value will give more weight to the mesh constraint. "
              "The mesh residuals printed at the end can be used to examine "
              "the effect of this weight.");

DEFINE_double(mesh_robust_threshold, 3.0,
              "A larger value will try harder to minimize large divergences from "
              "the mesh (but note that some of those may be outliers).");

DEFINE_bool(verbose, false,
            "Print the residuals and save the images and match files."
            "Stereo Pipeline's viewer can be used for visualizing these.");

namespace dense_map {

  // Calculate interpolated world to camera trans
  Eigen::Affine3d calc_world_to_cam_trans(const double* beg_world_to_ref_t,
                                          const double* end_world_to_ref_t,
                                          const double* ref_to_cam_trans,
                                          double beg_ref_stamp,
                                          double end_ref_stamp,
                                          double ref_to_cam_offset,
                                          double cam_stamp) {
    Eigen::Affine3d beg_world_to_ref_aff;
    array_to_rigid_transform(beg_world_to_ref_aff, beg_world_to_ref_t);

    Eigen::Affine3d end_world_to_ref_aff;
    array_to_rigid_transform(end_world_to_ref_aff, end_world_to_ref_t);

    Eigen::Affine3d ref_to_cam_aff;
    array_to_rigid_transform(ref_to_cam_aff, ref_to_cam_trans);

    // std::cout.precision(18);
    // std::cout << "--beg stamp " << beg_ref_stamp << std::endl;
    // std::cout << "--end stamp " << end_ref_stamp << std::endl;
    // std::cout << "cam stamp   " << cam_stamp << std::endl;
    // std::cout << "ref to cam off " << ref_to_cam_offset << std::endl;
    // std::cout << "--ref to cam trans\n" << ref_to_cam_aff.matrix() << std::endl;

    // Covert from cam time to ref time and normalize
    double alpha = (cam_stamp - ref_to_cam_offset - beg_ref_stamp)
      / (end_ref_stamp - beg_ref_stamp);

    if (beg_ref_stamp == end_ref_stamp)
      alpha = 0.0;  // handle division by zero

    // std::cout << "--alpha " << alpha << std::endl;
    if (alpha < 0.0 || alpha > 1.0) LOG(FATAL) << "Out of bounds in interpolation.\n";

    // Interpolate at desired time
    Eigen::Affine3d interp_world_to_ref_aff
      = dense_map::linearInterp(alpha, beg_world_to_ref_aff, end_world_to_ref_aff);

    Eigen::Affine3d interp_world_to_cam_afff = ref_to_cam_aff * interp_world_to_ref_aff;

    // std::cout << "final trans\n" << interp_world_to_cam_afff.matrix() << std::endl;

    return interp_world_to_cam_afff;
  }

  // TODO(oalexan1): Store separately matches which end up being
  // squashed in a pid_cid_to_fid.

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

// An error function minimizing the error of projecting
// an xyz point into a camera that is bracketed by
// two reference cameras. The precise timestamp offset
// between them is also floated.
struct BracketedCamError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BracketedCamError(Eigen::Vector2d const& meas_dist_pix,
                    double left_ref_stamp, double right_ref_stamp, double cam_stamp,
                    std::vector<int> const& block_sizes,
                    camera::CameraParameters const& cam_params):
    m_meas_dist_pix(meas_dist_pix),
    m_left_ref_stamp(left_ref_stamp),
    m_right_ref_stamp(right_ref_stamp),
    m_cam_stamp(cam_stamp),
    m_block_sizes(block_sizes),
    m_cam_params(cam_params),
    m_num_focal_lengths(1) {
    // Sanity check
    if (m_block_sizes.size() != 8 || m_block_sizes[0] != NUM_RIGID_PARAMS ||
        m_block_sizes[1] != NUM_RIGID_PARAMS || m_block_sizes[2] != NUM_RIGID_PARAMS ||
        m_block_sizes[3] != NUM_XYZ_PARAMS || m_block_sizes[4] != NUM_SCALAR_PARAMS ||
        m_block_sizes[5] != m_num_focal_lengths || m_block_sizes[6] != NUM_OPT_CTR_PARAMS ||
        m_block_sizes[7] != 1  // This will be overwritten shortly
    ) {
      LOG(FATAL) << "BracketedCamError: The block sizes were not set up properly.\n";
    }

    // Set the correct distortion size. This cannot be done in the interface for now.
    // TODO(oalexan1): Make individual block sizes for each camera, then this issue will go away.
    m_block_sizes[7] = m_cam_params.GetDistortion().size();
  }

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  bool operator()(double const* const* parameters, double* residuals) const {
    Eigen::Affine3d world_to_cam_trans =
      calc_world_to_cam_trans(parameters[0],  // beg_world_to_ref_t
                              parameters[1],  // end_world_to_ref_t
                              parameters[2],  // ref_to_cam_trans
                              m_left_ref_stamp, m_right_ref_stamp,
                              parameters[4][0],  // ref_to_cam_offset
                              m_cam_stamp);

    // World point
    Eigen::Vector3d X(parameters[3][0], parameters[3][1], parameters[3][2]);
    // std::cout << "--bracketX is " << X.transpose() << std::endl;

    // Make a deep copy which we will modify
    camera::CameraParameters cam_params = m_cam_params;
    Eigen::Vector2d focal_vector = Eigen::Vector2d(parameters[5][0], parameters[5][0]);
    Eigen::Vector2d optical_center(parameters[6][0], parameters[6][1]);
    Eigen::VectorXd distortion(m_block_sizes[7]);
    for (int i = 0; i < m_block_sizes[7]; i++) distortion[i] = parameters[7][i];
    cam_params.SetFocalLength(focal_vector);
    cam_params.SetOpticalOffset(optical_center);
    cam_params.SetDistortion(distortion);

    // std::cout << "--focal vector " << focal_vector.transpose() << std::endl;
    // std::cout << "--opt ctr " << optical_center.transpose() << std::endl;
    // std::cout << "-dist " << distortion.transpose() << std::endl;

    // Convert world point to given cam coordinates
    X = world_to_cam_trans * X;

    // std::cout << "--trans X " << X.transpose() << std::endl;

    // Project into the image
    Eigen::Vector2d undist_pix = cam_params.GetFocalVector().cwiseProduct(X.hnormalized());
    Eigen::Vector2d curr_dist_pix;
    cam_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undist_pix, &curr_dist_pix);

    // Compute the residuals
    residuals[0] = curr_dist_pix[0] - m_meas_dist_pix[0];
    residuals[1] = curr_dist_pix[1] - m_meas_dist_pix[1];

    // std::cout << "--residuals " << residuals[0] << ' ' << residuals[1] << std::endl;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction*
  Create(Eigen::Vector2d const& meas_dist_pix, double left_ref_stamp, double right_ref_stamp,
         double cam_stamp, std::vector<int> const& block_sizes,
         camera::CameraParameters const& cam_params) {
    ceres::DynamicNumericDiffCostFunction<BracketedCamError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<BracketedCamError>
      (new BracketedCamError(meas_dist_pix, left_ref_stamp, right_ref_stamp,
                             cam_stamp, block_sizes, cam_params));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_RESIDUALS);

    // The camera wrapper knows all of the block sizes to add, except
    // for distortion, which is last
    for (size_t i = 0; i + 1 < block_sizes.size(); i++)  //  note the i + 1
      cost_function->AddParameterBlock(block_sizes[i]);

    // The distortion block size is added separately as it is variable
    cost_function->AddParameterBlock(cam_params.GetDistortion().size());

    return cost_function;
  }

 private:
  Eigen::Vector2d m_meas_dist_pix;             // Measured distorted current camera pixel
  double m_left_ref_stamp, m_right_ref_stamp;  // left and right ref cam timestamps
  double m_cam_stamp;                          // Current cam timestamp
  std::vector<int> m_block_sizes;
  camera::CameraParameters m_cam_params;
  int m_num_focal_lengths;
};  // End class BracketedCamError

// An error function minimizing the error of projecting an xyz point
// into a reference camera. Bracketing, timestamps, and transform to
// ref cam are not needed.
struct RefCamError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RefCamError(Eigen::Vector2d const& meas_dist_pix,
              std::vector<int> const& block_sizes,
              camera::CameraParameters const& cam_params):
    m_meas_dist_pix(meas_dist_pix),
    m_block_sizes(block_sizes),
    m_cam_params(cam_params),
    m_num_focal_lengths(1) {
    // Sanity check
    if (m_block_sizes.size() != 5 ||
        m_block_sizes[0] != NUM_RIGID_PARAMS ||
        m_block_sizes[1] != NUM_XYZ_PARAMS ||
        m_block_sizes[2] != m_num_focal_lengths ||
        m_block_sizes[3] != NUM_OPT_CTR_PARAMS ||
        m_block_sizes[4] != 1  // This will be overwritten shortly
    ) {
      LOG(FATAL) << "RefCamError: The block sizes were not set up properly.\n";
    }

    // Set the correct distortion size. This cannot be done in the interface for now.

    // TODO(oalexan1): Make individual block sizes for each camera,
    // then this issue will go away.
    m_block_sizes[4] = m_cam_params.GetDistortion().size();
  }

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  bool operator()(double const* const* parameters, double* residuals) const {
    Eigen::Affine3d world_to_ref_t;
    array_to_rigid_transform(world_to_ref_t, parameters[0]);

    // World point
    Eigen::Vector3d X;
    for (int it = 0; it < NUM_XYZ_PARAMS; it++) X[it] = parameters[1][it];
    // std::cout << "--refX is " << X.transpose() << std::endl;

    // Make a deep copy which we will modify
    camera::CameraParameters cam_params = m_cam_params;
    Eigen::Vector2d focal_vector = Eigen::Vector2d(parameters[2][0], parameters[2][0]);
    Eigen::Vector2d optical_center(parameters[3][0], parameters[3][1]);
    Eigen::VectorXd distortion(m_block_sizes[4]);
    for (int i = 0; i < m_block_sizes[4]; i++) distortion[i] = parameters[4][i];
    cam_params.SetFocalLength(focal_vector);
    cam_params.SetOpticalOffset(optical_center);
    cam_params.SetDistortion(distortion);

    // std::cout << "--focal vector " << focal_vector.transpose() << std::endl;
    // std::cout << "--opt ctr " << optical_center.transpose() << std::endl;
    // std::cout << "-dist " << distortion.transpose() << std::endl;

    // Convert world point to given cam coordinates
    X = world_to_ref_t * X;

    // std::cout << "--trans X " << X.transpose() << std::endl;

    // Project into the image
    Eigen::Vector2d undist_pix = cam_params.GetFocalVector().cwiseProduct(X.hnormalized());
    Eigen::Vector2d curr_dist_pix;
    cam_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undist_pix, &curr_dist_pix);

    // Compute the residuals
    residuals[0] = curr_dist_pix[0] - m_meas_dist_pix[0];
    residuals[1] = curr_dist_pix[1] - m_meas_dist_pix[1];

    // std::cout << "--residuals " << residuals[0] << ' ' << residuals[1] << std::endl;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction*
  Create(Eigen::Vector2d const& meas_dist_pix,
         std::vector<int> const& block_sizes,
         camera::CameraParameters const& cam_params) {
    ceres::DynamicNumericDiffCostFunction<RefCamError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<RefCamError>
      (new RefCamError(meas_dist_pix, block_sizes, cam_params));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_RESIDUALS);

    // The camera wrapper knows all of the block sizes to add, except
    // for distortion, which is last
    for (size_t i = 0; i + 1 < block_sizes.size(); i++)  // note the i + 1
      cost_function->AddParameterBlock(block_sizes[i]);

    // The distortion block size is added separately as it is variable
    cost_function->AddParameterBlock(cam_params.GetDistortion().size());

    return cost_function;
  }

 private:
  Eigen::Vector2d m_meas_dist_pix;
  std::vector<int> m_block_sizes;
  camera::CameraParameters m_cam_params;
  int m_num_focal_lengths;
};  // End class RefCamError

// An error function minimizing the product of a given weight and the
// error between a triangulated point and a measured depth point.  The
// depth point needs to be transformed to world coordinates first. For
// that one has to do pose interpolation.
struct BracketedDepthError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BracketedDepthError(double weight, Eigen::Vector3d const& meas_depth_xyz,
                      double left_ref_stamp, double right_ref_stamp, double cam_stamp,
                      std::vector<int> const& block_sizes):
    m_weight(weight),
    m_meas_depth_xyz(meas_depth_xyz),
    m_left_ref_stamp(left_ref_stamp),
    m_right_ref_stamp(right_ref_stamp),
    m_cam_stamp(cam_stamp),
    m_block_sizes(block_sizes) {
    // Sanity check
    if (m_block_sizes.size() != 7 ||
        m_block_sizes[0] != NUM_RIGID_PARAMS  ||
        m_block_sizes[1] != NUM_RIGID_PARAMS  ||
        m_block_sizes[2] != NUM_RIGID_PARAMS  ||
        m_block_sizes[3] != NUM_RIGID_PARAMS  ||
        m_block_sizes[4] != NUM_SCALAR_PARAMS ||
        m_block_sizes[5] != NUM_XYZ_PARAMS    ||
        m_block_sizes[6] != NUM_SCALAR_PARAMS) {
      LOG(FATAL) << "BracketedDepthError: The block sizes were not set up properly.\n";
    }
  }

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  bool operator()(double const* const* parameters, double* residuals) const {
    // Current world to camera transform
    Eigen::Affine3d world_to_cam_trans =
      calc_world_to_cam_trans(parameters[0],  // beg_world_to_ref_t
                              parameters[1],  // end_world_to_ref_t
                              parameters[2],  // ref_to_cam_trans
                              m_left_ref_stamp, m_right_ref_stamp,
                              parameters[6][0],  // ref_to_cam_offset
                              m_cam_stamp);

    // The current transform from the depth point cloud to the camera image
    Eigen::Affine3d depth_to_image;
    array_to_rigid_transform(depth_to_image, parameters[3]);

    // Apply the scale
    double depth_to_image_scale = parameters[4][0];
    depth_to_image.linear() *= depth_to_image_scale;
    // std::cout << "--depth to image:\n" << depth_to_image.matrix() << std::endl;

    // std::cout << "--meas pt " << m_meas_depth_xyz.transpose() << std::endl;

    // Convert from depth cloud coordinates to cam coordinates
    Eigen::Vector3d M = depth_to_image * m_meas_depth_xyz;

    // std::cout << "--image meas pt " << M.transpose() << std::endl;

    // Convert to world coordinates
    M = world_to_cam_trans.inverse() * M;
    // std::cout << "--depth in world coords " << M.transpose() << std::endl;

    // Triangulated world point
    Eigen::Vector3d X(parameters[5][0], parameters[5][1], parameters[5][2]);
    // std::cout << "--triangulated X is " << X.transpose() << std::endl;

    // std::cout << "--weight " << m_weight << std::endl;

    // Compute the residuals
    for (size_t it = 0; it < NUM_XYZ_PARAMS; it++) {
      residuals[it] = m_weight * (X[it] - M[it]);
      // std::cout << "--residual " << residuals[it] << std::endl;
    }

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(double weight, Eigen::Vector3d const& meas_depth_xyz,
                                     double left_ref_stamp, double right_ref_stamp,
                                     double cam_stamp, std::vector<int> const& block_sizes) {
    ceres::DynamicNumericDiffCostFunction<BracketedDepthError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<BracketedDepthError>
      (new BracketedDepthError(weight, meas_depth_xyz, left_ref_stamp, right_ref_stamp,
                             cam_stamp, block_sizes));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_XYZ_PARAMS);

    for (size_t i = 0; i < block_sizes.size(); i++)
      cost_function->AddParameterBlock(block_sizes[i]);

    return cost_function;
  }

 private:
  double m_weight;                             // How much weight to give to this constraint
  Eigen::Vector3d m_meas_depth_xyz;            // Measured depth measurement
  double m_left_ref_stamp, m_right_ref_stamp;  // left and right ref cam timestamps
  double m_cam_stamp;                          // Current cam timestamp
  std::vector<int> m_block_sizes;
};  // End class BracketedDepthError

// An error function minimizing the error of projection of haz cam
// depth points to the haz cam image.
struct DepthToHazError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DepthToHazError(Eigen::Vector2d const& haz_pix, Eigen::Vector3d const& depth_xyz,
                  std::vector<int> const& block_sizes,
                  camera::CameraParameters const& haz_cam_params)
    : m_haz_pix(haz_pix), m_depth_xyz(depth_xyz), m_block_sizes(block_sizes),
      m_haz_cam_params(haz_cam_params) {
    // Sanity check.
    if (m_block_sizes.size() != 2 || m_block_sizes[0] != NUM_RIGID_PARAMS || m_block_sizes[1] != NUM_SCALAR_PARAMS) {
      LOG(FATAL) << "DepthToHazError: The block sizes were not set up properly.\n";
    }
  }

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  // Takes array of arrays as parameters.
  bool operator()(double const* const* parameters, double* residuals) const {
    // Populate the intrinsics

    // The current transform from the depth point cloud to the image
    Eigen::Affine3d depth_to_image;
    array_to_rigid_transform(depth_to_image, parameters[0]);

    // Apply the scale
    double depth_to_image_scale = parameters[1][0];
    depth_to_image.linear() *= depth_to_image_scale;

    // Convert from depth cloud coordinates to haz cam coordinates
    Eigen::Vector3d X = depth_to_image * m_depth_xyz;

    // Project into the camera
    Eigen::Vector2d undist_pix = m_haz_cam_params.GetFocalVector().cwiseProduct(X.hnormalized());
    // Eigen::Vector2d dist_pix;
    // m_haz_cam_params.Convert<camera::UNDISTORTED_C,
    // camera::DISTORTED>(undist_pix, &dist_pix);

    // Compute the residuals
    residuals[0] = undist_pix[0] - m_haz_pix[0];
    residuals[1] = undist_pix[1] - m_haz_pix[1];

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Eigen::Vector2d const& nav_pix, Eigen::Vector3d const& depth_xyz,
                                     std::vector<int> const& block_sizes,
                                     camera::CameraParameters const& haz_cam_params) {
    ceres::DynamicNumericDiffCostFunction<DepthToHazError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<DepthToHazError>(
        new DepthToHazError(nav_pix, depth_xyz, block_sizes, haz_cam_params));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_RESIDUALS);

    // The camera wrapper knows all of the block sizes to add.
    for (size_t i = 0; i < block_sizes.size(); i++) {
      cost_function->AddParameterBlock(block_sizes[i]);
    }
    return cost_function;
  }

 private:
  Eigen::Vector2d m_haz_pix;    // The pixel observation
  Eigen::Vector3d m_depth_xyz;  // The measured position
  std::vector<int> m_block_sizes;
  camera::CameraParameters m_haz_cam_params;
};  // End class DepthToHazError

// An error function minimizing the error of projection of a point in the nav cam
// image. Both the point and the camera pose are variables of optimization.
struct NavError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NavError(Eigen::Vector2d const& nav_pix, std::vector<int> const& block_sizes,
           camera::CameraParameters const& nav_cam_params)
      : m_nav_pix(nav_pix), m_block_sizes(block_sizes), m_nav_cam_params(nav_cam_params) {
    // Sanity check
    if (m_block_sizes.size() != 2 || m_block_sizes[0] != NUM_RIGID_PARAMS || m_block_sizes[1] != NUM_XYZ_PARAMS) {
      LOG(FATAL) << "NavError: The block sizes were not set up properly.\n";
    }
  }

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  // Takes array of arrays as parameters.
  bool operator()(double const* const* parameters, double* residuals) const {
    // Populate the intrinsics

    Eigen::Affine3d world_to_nav_cam;
    array_to_rigid_transform(world_to_nav_cam, parameters[0]);

    Eigen::Vector3d xyz;
    for (int it = 0; it < NUM_XYZ_PARAMS; it++) xyz[it] = parameters[1][it];

    // Convert to camera coordinates
    Eigen::Vector3d X = world_to_nav_cam * xyz;

    // Project into the camera
    Eigen::Vector2d undist_pix = m_nav_cam_params.GetFocalVector().cwiseProduct(X.hnormalized());
    // Eigen::Vector2d dist_pix;
    // m_nav_cam_params.Convert<camera::UNDISTORTED_C,
    // camera::DISTORTED_C>(undist_pix, &dist_pix);

    // Compute the residuals
    residuals[0] = undist_pix[0] - m_nav_pix[0];
    residuals[1] = undist_pix[1] - m_nav_pix[1];

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Eigen::Vector2d const& nav_pix, std::vector<int> const& block_sizes,
                                     camera::CameraParameters const& nav_cam_params) {
    ceres::DynamicNumericDiffCostFunction<NavError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<NavError>(new NavError(nav_pix, block_sizes, nav_cam_params));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_RESIDUALS);

    // The camera wrapper knows all of the block sizes to add.
    for (size_t i = 0; i < block_sizes.size(); i++) {
      cost_function->AddParameterBlock(block_sizes[i]);
    }
    return cost_function;
  }

 private:
  Eigen::Vector2d m_nav_pix;  // The pixel observation
  std::vector<int> m_block_sizes;
  camera::CameraParameters m_nav_cam_params;
};  // End class NavError

// An error function minimizing the error of transforming depth points
// first to haz cam image coordinates, then to nav cam coordinates
// (via time interpolation) then to world coordinates, then by
// projecting into the nav cam image having a match point with the
// original haz cam image. This is a little tricky to follow, because
// each haz cam image is bound in time by two nav cam images.
struct DepthToNavError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DepthToNavError(Eigen::Vector2d const& nav_pix, Eigen::Vector3d const& depth_xyz,
                  double alpha,  // used for interpolation
                  bool match_left, std::vector<int> const& block_sizes, camera::CameraParameters const& nav_cam_params)
      : m_nav_pix(nav_pix),
        m_depth_xyz(depth_xyz),
        m_alpha(alpha),
        m_match_left(match_left),
        m_block_sizes(block_sizes),
        m_nav_cam_params(nav_cam_params) {
    // Sanity check.
    if (m_block_sizes.size() != 5 || m_block_sizes[0] != NUM_RIGID_PARAMS || m_block_sizes[1] != NUM_RIGID_PARAMS ||
        m_block_sizes[2] != NUM_RIGID_PARAMS || m_block_sizes[3] != NUM_RIGID_PARAMS ||
        m_block_sizes[4] != NUM_SCALAR_PARAMS) {
      LOG(FATAL) << "DepthToNavError: The block sizes were not set up properly.\n";
    }
  }

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  bool operator()(double const* const* parameters, double* residuals) const {
    // As mentioned before, we have to deal with four transforms

    Eigen::Affine3d left_nav_trans;
    array_to_rigid_transform(left_nav_trans, parameters[0]);

    Eigen::Affine3d right_nav_trans;
    array_to_rigid_transform(right_nav_trans, parameters[1]);

    Eigen::Affine3d hazcam_to_navcam_trans;
    array_to_rigid_transform(hazcam_to_navcam_trans, parameters[2]);

    // The haz cam depth to image transform, which has a fixed scale
    Eigen::Affine3d hazcam_depth_to_image_trans;
    array_to_rigid_transform(hazcam_depth_to_image_trans, parameters[3]);
    double depth_to_image_scale = parameters[4][0];
    hazcam_depth_to_image_trans.linear() *= depth_to_image_scale;

    // Convert from depth cloud coordinates to haz cam coordinates
    Eigen::Vector3d X = hazcam_depth_to_image_trans * m_depth_xyz;

    // Convert to nav cam coordinates
    X = hazcam_to_navcam_trans * X;

    // The haz cam to nav cam transform at the haz cam time is obtained
    // by interpolation in time
    Eigen::Affine3d interp_world_to_nav_trans = dense_map::linearInterp(m_alpha, left_nav_trans, right_nav_trans);

    // Convert to world coordinates
    X = interp_world_to_nav_trans.inverse() * X;

    // Transform to either the left or right nav camera coordinates,
    // depending on for which one we managed to find a match with he
    // haz cam image
    if (m_match_left) {
      X = left_nav_trans * X;
    } else {
      X = right_nav_trans * X;
    }

    // Project into the image
    Eigen::Vector2d undist_pix = m_nav_cam_params.GetFocalVector().cwiseProduct(X.hnormalized());
    // Eigen::Vector2d dist_pix;
    // m_nav_cam_params.Convert<camera::UNDISTORTED_C,
    // camera::DISTORTED>(undist_pix, &dist_pix);

    // Compute the residuals
    residuals[0] = undist_pix[0] - m_nav_pix[0];
    residuals[1] = undist_pix[1] - m_nav_pix[1];

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Eigen::Vector2d const& nav_pix, Eigen::Vector3d const& depth_xyz, double alpha,
                                     bool match_left, std::vector<int> const& block_sizes,
                                     camera::CameraParameters const& nav_cam_params) {
    ceres::DynamicNumericDiffCostFunction<DepthToNavError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<DepthToNavError>(
        new DepthToNavError(nav_pix, depth_xyz, alpha, match_left, block_sizes, nav_cam_params));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_RESIDUALS);

    // The camera wrapper knows all of the block sizes to add.
    for (size_t i = 0; i < block_sizes.size(); i++) {
      cost_function->AddParameterBlock(block_sizes[i]);
    }
    return cost_function;
  }

 private:
  Eigen::Vector2d m_nav_pix;    // The nav cam pixel observation
  Eigen::Vector3d m_depth_xyz;  // The measured position in the depth camera
  double m_alpha;
  bool m_match_left;
  std::vector<int> m_block_sizes;
  camera::CameraParameters m_nav_cam_params;
};  // End class DepthToNavError

// An error function minimizing the error of transforming depth points at haz cam
// time through enough coordinate systems until it can project
// into the sci cam at the sci cam time
struct DepthToSciError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DepthToSciError(Eigen::Vector2d const& dist_sci_pix, Eigen::Vector3d const& depth_xyz, double alpha_haz,
                  double alpha_sci,  // used for interpolation
                  std::vector<int> const& block_sizes, camera::CameraParameters const& sci_cam_params)
      : m_dist_sci_pix(dist_sci_pix),
        m_depth_xyz(depth_xyz),
        m_alpha_haz(alpha_haz),
        m_alpha_sci(alpha_sci),
        m_block_sizes(block_sizes),
        m_sci_cam_params(sci_cam_params),
        m_num_focal_lengths(1) {
    // Sanity check.
    if (m_block_sizes.size() != 9 || m_block_sizes[0] != NUM_RIGID_PARAMS || m_block_sizes[1] != NUM_RIGID_PARAMS ||
        m_block_sizes[2] != NUM_RIGID_PARAMS || m_block_sizes[3] != NUM_RIGID_PARAMS ||
        m_block_sizes[4] != NUM_RIGID_PARAMS || m_block_sizes[5] != NUM_SCALAR_PARAMS ||
        m_block_sizes[6] != m_num_focal_lengths || m_block_sizes[7] != NUM_OPT_CTR_PARAMS ||
        m_block_sizes[8] != m_sci_cam_params.GetDistortion().size()) {
      LOG(FATAL) << "DepthToSciError: The block sizes were not set up properly.\n";
    }
  }

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  bool operator()(double const* const* parameters, double* residuals) const {
    // Make a deep copy which we will modify
    camera::CameraParameters sci_cam_params = m_sci_cam_params;

    // We have to deal with four transforms
    Eigen::Affine3d left_nav_trans;
    array_to_rigid_transform(left_nav_trans, parameters[0]);

    Eigen::Affine3d right_nav_trans;
    array_to_rigid_transform(right_nav_trans, parameters[1]);

    Eigen::Affine3d hazcam_to_navcam_trans;
    array_to_rigid_transform(hazcam_to_navcam_trans, parameters[2]);

    Eigen::Affine3d scicam_to_hazcam_trans;
    array_to_rigid_transform(scicam_to_hazcam_trans, parameters[3]);

    // The haz cam depth to image transform, which has a fixed scale
    Eigen::Affine3d hazcam_depth_to_image_trans;
    array_to_rigid_transform(hazcam_depth_to_image_trans, parameters[4]);
    double depth_to_image_scale = parameters[5][0];
    hazcam_depth_to_image_trans.linear() *= depth_to_image_scale;

    // Intrinsics, including a single focal length
    Eigen::Vector2d focal_vector = Eigen::Vector2d(parameters[6][0], parameters[6][0]);
    Eigen::Vector2d optical_center(parameters[7][0], parameters[7][1]);
    Eigen::VectorXd distortion(m_block_sizes[8]);
    for (int i = 0; i < m_block_sizes[8]; i++) distortion[i] = parameters[8][i];
    sci_cam_params.SetFocalLength(focal_vector);
    sci_cam_params.SetOpticalOffset(optical_center);
    sci_cam_params.SetDistortion(distortion);

    // Convert from depth cloud coordinates to haz cam coordinates
    Eigen::Vector3d X = hazcam_depth_to_image_trans * m_depth_xyz;

    // Convert to nav cam coordinates at haz cam time
    X = hazcam_to_navcam_trans * X;

    // World to navcam at haz time
    Eigen::Affine3d interp_world_to_nav_trans_haz_time =
      dense_map::linearInterp(m_alpha_haz, left_nav_trans, right_nav_trans);

    // World to nav time at sci time
    Eigen::Affine3d interp_world_to_nav_trans_sci_time =
      dense_map::linearInterp(m_alpha_sci, left_nav_trans, right_nav_trans);

    // Convert to world coordinates
    X = interp_world_to_nav_trans_haz_time.inverse() * X;

    // Convert to nav coordinates at sci cam time
    X = interp_world_to_nav_trans_sci_time * X;

    // Convert to sci cam coordinates
    X = scicam_to_hazcam_trans.inverse() * hazcam_to_navcam_trans.inverse() * X;

    // Convert to sci cam pix
    Eigen::Vector2d undist_pix = sci_cam_params.GetFocalVector().cwiseProduct(X.hnormalized());

    // Apply distortion
    Eigen::Vector2d comp_dist_sci_pix;
    sci_cam_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undist_pix, &comp_dist_sci_pix);

    // Compute the residuals
    residuals[0] = comp_dist_sci_pix[0] - m_dist_sci_pix[0];
    residuals[1] = comp_dist_sci_pix[1] - m_dist_sci_pix[1];

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Eigen::Vector2d const& dist_sci_pix, Eigen::Vector3d const& depth_xyz,
                                     double alpha_haz, double alpha_sci, std::vector<int> const& block_sizes,
                                     camera::CameraParameters const& sci_cam_params) {
    ceres::DynamicNumericDiffCostFunction<DepthToSciError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<DepthToSciError>(
        new DepthToSciError(dist_sci_pix, depth_xyz, alpha_haz, alpha_sci, block_sizes, sci_cam_params));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_RESIDUALS);

    // The camera wrapper knows all of the block sizes to add.
    for (size_t i = 0; i < block_sizes.size(); i++) {
      cost_function->AddParameterBlock(block_sizes[i]);
    }
    return cost_function;
  }

 private:
  Eigen::Vector2d m_dist_sci_pix;  // The sci cam pixel observation
  Eigen::Vector3d m_depth_xyz;     // The measured position in the depth camera
  double m_alpha_haz;
  double m_alpha_sci;
  std::vector<int> m_block_sizes;
  camera::CameraParameters m_sci_cam_params;
  int m_num_focal_lengths;
};  // End class DepthToSciError

// An error function projecting an xyz point in the sci cam
// bounded by two nav cams.
struct SciError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SciError(Eigen::Vector2d const& dist_sci_pix,
           double alpha,  // used for interpolation
           std::vector<int> const& block_sizes, camera::CameraParameters const& sci_cam_params)
      : m_dist_sci_pix(dist_sci_pix),
        m_alpha(alpha),
        m_block_sizes(block_sizes),
        m_sci_cam_params(sci_cam_params),
        m_num_focal_lengths(1) {
    // Sanity check.
    if (m_block_sizes.size() != 8 ||
        m_block_sizes[0] != NUM_RIGID_PARAMS ||
        m_block_sizes[1] != NUM_RIGID_PARAMS ||
        m_block_sizes[2] != NUM_RIGID_PARAMS ||
        m_block_sizes[3] != NUM_RIGID_PARAMS ||
        m_block_sizes[4] != NUM_XYZ_PARAMS ||
        m_block_sizes[5] != m_num_focal_lengths ||
        m_block_sizes[6] != NUM_OPT_CTR_PARAMS ||
        m_block_sizes[7] != m_sci_cam_params.GetDistortion().size()) {
      LOG(FATAL) << "SciError: The block sizes were not set up properly.\n";
    }
  }

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  bool operator()(double const* const* parameters, double* residuals) const {
    // Make a deep copy which we will modify
    camera::CameraParameters sci_cam_params = m_sci_cam_params;

    // We have to deal with four transforms

    Eigen::Affine3d left_nav_trans;
    array_to_rigid_transform(left_nav_trans, parameters[0]);

    Eigen::Affine3d right_nav_trans;
    array_to_rigid_transform(right_nav_trans, parameters[1]);

    Eigen::Affine3d interp_world_to_nav_trans = dense_map::linearInterp(m_alpha, left_nav_trans, right_nav_trans);

    Eigen::Affine3d hazcam_to_navcam_trans;
    array_to_rigid_transform(hazcam_to_navcam_trans, parameters[2]);

    Eigen::Affine3d scicam_to_hazcam_trans;
    array_to_rigid_transform(scicam_to_hazcam_trans, parameters[3]);

    Eigen::Vector3d X;
    for (int it = 0; it < NUM_XYZ_PARAMS; it++) X[it] = parameters[4][it];

    // Intrinsics, including a single focal length
    Eigen::Vector2d focal_vector = Eigen::Vector2d(parameters[5][0], parameters[5][0]);
    Eigen::Vector2d optical_center(parameters[6][0], parameters[6][1]);
    Eigen::VectorXd distortion(m_block_sizes[7]);
    for (int i = 0; i < m_block_sizes[7]; i++) distortion[i] = parameters[7][i];
    sci_cam_params.SetFocalLength(focal_vector);
    sci_cam_params.SetOpticalOffset(optical_center);
    sci_cam_params.SetDistortion(distortion);

    // Find the sci cam to world transform
    Eigen::Affine3d interp_world_to_sci_trans =
      scicam_to_hazcam_trans.inverse() * hazcam_to_navcam_trans.inverse() * interp_world_to_nav_trans;

    // Project into the sci cam
    Eigen::Vector3d sciX = interp_world_to_sci_trans * X;
    Eigen::Vector2d undist_sci_pix = sci_cam_params.GetFocalVector().cwiseProduct(sciX.hnormalized());

    // Convert to distorted pixel
    Eigen::Vector2d comp_dist_sci_pix;
    sci_cam_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undist_sci_pix, &comp_dist_sci_pix);

    // Compute the residuals
    residuals[0] = comp_dist_sci_pix[0] - m_dist_sci_pix[0];
    residuals[1] = comp_dist_sci_pix[1] - m_dist_sci_pix[1];

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Eigen::Vector2d const& dist_sci_pix, double alpha,
                                     std::vector<int> const& block_sizes,
                                     camera::CameraParameters const& sci_cam_params) {
    ceres::DynamicNumericDiffCostFunction<SciError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<SciError>(
        new SciError(dist_sci_pix, alpha, block_sizes, sci_cam_params));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_RESIDUALS);

    // The camera wrapper knows all of the block sizes to add.
    for (size_t i = 0; i < block_sizes.size(); i++) {
      cost_function->AddParameterBlock(block_sizes[i]);
    }
    return cost_function;
  }

 private:
  Eigen::Vector2d m_dist_sci_pix;  // The sci cam pixel observation
  double m_alpha;
  std::vector<int> m_block_sizes;
  camera::CameraParameters m_sci_cam_params;
  int m_num_focal_lengths;
};  // End class SciError

// An error function minimizing a weight times the distance from a
// variable xyz point to a fixed reference xyz point.
struct XYZError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  XYZError(Eigen::Vector3d const& ref_xyz, std::vector<int> const& block_sizes, double weight)
      : m_ref_xyz(ref_xyz), m_block_sizes(block_sizes), m_weight(weight) {
    // Sanity check
    if (m_block_sizes.size() != 1 || m_block_sizes[0] != NUM_XYZ_PARAMS)
      LOG(FATAL) << "XYZError: The block sizes were not set up properly.\n";
  }

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  // Takes array of arrays as parameters.
  // TODO(oalexan1): May want to use the analytical Ceres cost function
  bool operator()(double const* const* parameters, double* residuals) const {
    // Compute the residuals
    for (int it = 0; it < NUM_XYZ_PARAMS; it++) residuals[it] = m_weight * (parameters[0][it] - m_ref_xyz[it]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Eigen::Vector3d const& ref_xyz,
                                     std::vector<int> const& block_sizes,
                                     double weight) {
    ceres::DynamicNumericDiffCostFunction<XYZError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<XYZError>(new XYZError(ref_xyz, block_sizes, weight));

    // The residual size is always the same
    cost_function->SetNumResiduals(NUM_XYZ_PARAMS);

    // The camera wrapper knows all of the block sizes to add.
    for (size_t i = 0; i < block_sizes.size(); i++) {
      cost_function->AddParameterBlock(block_sizes[i]);
    }
    return cost_function;
  }

 private:
  Eigen::Vector3d m_ref_xyz;  // reference xyz
  std::vector<int> m_block_sizes;
  double m_weight;
};  // End class XYZError

enum imgType { NAV_CAM, SCI_CAM, HAZ_CAM };

// Calculate the rmse residual for each residual type.
void calc_median_residuals(std::vector<double> const& residuals,
                           std::vector<std::string> const& residual_names,
                           std::string const& tag) {
  size_t num = residuals.size();

  if (num != residual_names.size())
    LOG(FATAL) << "There must be as many residuals as residual names.";

  std::map<std::string, std::vector<double> > stats;
  for (size_t it = 0; it < residuals.size(); it++)
    stats[residual_names[it]] = std::vector<double>();  // initialize

  for (size_t it = 0; it < residuals.size(); it++)
    stats[residual_names[it]].push_back(std::abs(residuals[it]));

  std::cout << "The 25, 50 and 75 percentile residual stats " << tag << std::endl;
  for (auto it = stats.begin(); it != stats.end(); it++) {
    std::string const& name = it->first;
    std::vector<double> vals = stats[name];  // make a copy
    std::sort(vals.begin(), vals.end());

    int len = vals.size();

    int it1 = static_cast<int>(0.25 * len);
    int it2 = static_cast<int>(0.5 * len);
    int it3 = static_cast<int>(0.75 * len);

    if (len = 0)
      std::cout << name << ": " << "none" << std::endl;
    else
      std::cout << name << ": " << vals[it1] << ' ' << vals[it2] << ' '
                << vals[it3] << std::endl;
  }
}

  // Intersect ray with mesh. Return true on success.
  bool ray_mesh_intersect(Eigen::Vector2d const& undist_pix,
                          camera::CameraParameters const& cam_params,
                          Eigen::Affine3d const& world_to_cam,
                          mve::TriangleMesh::Ptr const& mesh,
                          std::shared_ptr<BVHTree> const& bvh_tree,
                          double min_ray_dist, double max_ray_dist,
                          // Output
                          Eigen::Vector3d& intersection) {
    // Initialize the output
    intersection = Eigen::Vector3d(0.0, 0.0, 0.0);

    // Ray from camera going through the pixel
    Eigen::Vector3d cam_ray(undist_pix.x() / cam_params.GetFocalVector()[0],
                            undist_pix.y() / cam_params.GetFocalVector()[1], 1.0);
    cam_ray.normalize();

    Eigen::Affine3d cam_to_world = world_to_cam.inverse();
    Eigen::Vector3d world_ray = cam_to_world.linear() * cam_ray;
    Eigen::Vector3d cam_ctr = cam_to_world.translation();

    // Set up the ray structure for the mesh
    BVHTree::Ray bvh_ray;
    bvh_ray.origin = dense_map::eigen_to_vec3f(cam_ctr);
    bvh_ray.dir = dense_map::eigen_to_vec3f(world_ray);
    bvh_ray.dir.normalize();

    bvh_ray.tmin = min_ray_dist;
    bvh_ray.tmax = max_ray_dist;

    // Intersect the ray with the mesh
    BVHTree::Hit hit;
    if (bvh_tree->intersect(bvh_ray, &hit)) {
      double cam_to_mesh_dist = hit.t;
      intersection = cam_ctr + cam_to_mesh_dist * world_ray;
      return true;
    }

    return false;
  }

  // Prevent the linter from messing up with the beautiful formatting below
  void add_haz_nav_cost                                                 // NOLINT
  (// Inputs                                                            // NOLINT
   int haz_it, int nav_it, int nav_cam_start,                           // NOLINT
   double navcam_to_hazcam_timestamp_offset,                            // NOLINT
   MATCH_PAIR                   const & match_pair,                     // NOLINT
   std::vector<double>          const & haz_cam_intensity_timestamps,   // NOLINT
   std::vector<double>          const & ref_timestamps,             // NOLINT
   std::map<int, int>           const & haz_cam_to_left_nav_cam_index,  // NOLINT
   std::map<int, int>           const & haz_cam_to_right_nav_cam_index, // NOLINT
   camera::CameraParameters     const & nav_cam_params,                 // NOLINT
   camera::CameraParameters     const & haz_cam_params,                 // NOLINT
   std::vector<int>             const & depth_to_nav_block_sizes,       // NOLINT
   std::vector<int>             const & depth_to_haz_block_sizes,       // NOLINT
   std::vector<Eigen::Affine3d> const & world_to_ref_t,             // NOLINT
   std::vector<cv::Mat>         const & depth_clouds,                   // NOLINT
   // Outputs                                                           // NOLINT
   std::vector<std::string>           & residual_names,                 // NOLINT
   double                             & hazcam_depth_to_image_scale,    // NOLINT
   std::vector<double>                & world_to_ref_vec,                    // NOLINT
   std::vector<double>                & hazcam_to_navcam_vec,           // NOLINT
   std::vector<double>                & hazcam_depth_to_image_vec,      // NOLINT
   ceres::Problem                     & problem) {                      // NOLINT
    // Figure out the two nav cam indices bounding the current haz cam
    // Must have sparse_map_timestamp + navcam_to_hazcam_timestamp_offset <= haz_timestamp
    // which must be < next sparse_map_timestamp + navcam_to_hazcam_timestamp_offset.
    bool match_left = false;
    if (haz_cam_intensity_timestamps[haz_it] >=
        ref_timestamps[nav_cam_start + nav_it] + navcam_to_hazcam_timestamp_offset) {
      match_left = true;
    } else {
      match_left = false;  // match right then
    }

    auto left_it = haz_cam_to_left_nav_cam_index.find(haz_it);
    auto right_it = haz_cam_to_right_nav_cam_index.find(haz_it);
    if (left_it == haz_cam_to_left_nav_cam_index.end() ||
        right_it == haz_cam_to_right_nav_cam_index.end())
      LOG(FATAL) << "Book-keeping error in add_haz_nav_cost.";

    int left_nav_it   = left_it->second;
    int right_nav_it  = right_it->second;

    if (nav_cam_start + right_nav_it >= static_cast<int>(ref_timestamps.size()) ||
        haz_it >= static_cast<int>(haz_cam_intensity_timestamps.size()))
      LOG(FATAL) << "Book-keeping error 2.";

    // Left and right nav cam image time, in haz_cam's time measurement
    double left_time = ref_timestamps[nav_cam_start + left_nav_it]
      + navcam_to_hazcam_timestamp_offset;
    double right_time = ref_timestamps[nav_cam_start + right_nav_it]
      + navcam_to_hazcam_timestamp_offset;
    double haz_time = haz_cam_intensity_timestamps[haz_it];

    bool good = (left_time <= haz_time && haz_time < right_time);

    if (!good) LOG(FATAL) << "Book-keeping error 3.";

    // The current nav it better be either the left or right kind
    if (nav_it != left_nav_it && nav_it != right_nav_it) LOG(FATAL) << "Book-keeping error 4.";

    // Find the transform from the world to nav cam at the haz cam time
    Eigen::Affine3d left_nav_trans = world_to_ref_t[nav_cam_start + left_nav_it];
    Eigen::Affine3d right_nav_trans = world_to_ref_t[nav_cam_start + right_nav_it];
    double alpha = (haz_time - left_time) / (right_time - left_time);
    if (right_time == left_time) alpha = 0.0;  // handle division by zero

    if (!FLAGS_timestamp_interpolation) alpha = round(alpha);

    Eigen::Affine3d world_to_nav_trans_at_haz_time
      = dense_map::linearInterp(alpha, left_nav_trans, right_nav_trans);

    std::vector<dense_map::InterestPoint> const& haz_ip_vec = match_pair.first;   // alias
    std::vector<dense_map::InterestPoint> const& nav_ip_vec = match_pair.second;  // alias

    cv::Mat const& depth_cloud = depth_clouds[haz_it];  // alias

    for (size_t ip_it = 0; ip_it < haz_ip_vec.size(); ip_it++) {
      // Find the haz cam depth measurement. Use nearest neighbor interpolation
      // to look into the depth cloud.
      int col = round(haz_ip_vec[ip_it].x);
      int row = round(haz_ip_vec[ip_it].y);

      if (col < 0 || row < 0 || col >= depth_cloud.cols || row >= depth_cloud.rows)
        LOG(FATAL) << "Book-keeping error 5.";

      // Skip any point that goes out of bounds due to rounding
      if (col == depth_cloud.cols || row == depth_cloud.rows) continue;

      cv::Vec3f cv_depth_xyz = depth_cloud.at<cv::Vec3f>(row, col);

      // Skip invalid measurements
      if (cv_depth_xyz == cv::Vec3f(0, 0, 0)) continue;

      // Convert to Eigen
      Eigen::Vector3d depth_xyz(cv_depth_xyz[0], cv_depth_xyz[1], cv_depth_xyz[2]);

      Eigen::Vector2d undist_haz_ip;
      Eigen::Vector2d undist_nav_ip;
      {
        // Make sure we don't use the distorted pixels from now on
        Eigen::Vector2d haz_ip(haz_ip_vec[ip_it].x, haz_ip_vec[ip_it].y);
        Eigen::Vector2d nav_ip(nav_ip_vec[ip_it].x, nav_ip_vec[ip_it].y);
        haz_cam_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(haz_ip, &undist_haz_ip);
        nav_cam_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(nav_ip, &undist_nav_ip);
      }

      // Ensure the depth point projects well into the haz cam interest point
      ceres::CostFunction* depth_to_haz_cost_function =
        dense_map::DepthToHazError::Create(undist_haz_ip, depth_xyz, depth_to_haz_block_sizes,
                                           haz_cam_params);
      ceres::LossFunction* depth_to_haz_loss_function
        = dense_map::GetLossFunction("cauchy", FLAGS_robust_threshold);
      problem.AddResidualBlock(depth_to_haz_cost_function, depth_to_haz_loss_function,
                               &hazcam_depth_to_image_vec[0],
                               &hazcam_depth_to_image_scale);
      residual_names.push_back("haznavhaz1");
      residual_names.push_back("haznavhaz2");

      // Ensure that the depth points projects well in the nav cam interest point
      ceres::CostFunction* depth_to_nav_cost_function
        = dense_map::DepthToNavError::Create(undist_nav_ip, depth_xyz, alpha, match_left,
                                             depth_to_nav_block_sizes, nav_cam_params);
      ceres::LossFunction* depth_to_nav_loss_function
        = dense_map::GetLossFunction("cauchy", FLAGS_robust_threshold);
      int left_nav_index  = NUM_RIGID_PARAMS * (nav_cam_start + left_nav_it);
      int right_nav_index = NUM_RIGID_PARAMS * (nav_cam_start + right_nav_it);
      problem.AddResidualBlock(depth_to_nav_cost_function, depth_to_nav_loss_function,
                               &world_to_ref_vec[left_nav_index],
                               &world_to_ref_vec[right_nav_index],
                               &hazcam_to_navcam_vec[0], &hazcam_depth_to_image_vec[0],
                               &hazcam_depth_to_image_scale);
      residual_names.push_back("haznavnav1");
      residual_names.push_back("haznavnav2");

      if (FLAGS_fix_map) {
        problem.SetParameterBlockConstant(&world_to_ref_vec[left_nav_index]);
        problem.SetParameterBlockConstant(&world_to_ref_vec[right_nav_index]);
      }
    }

    return;
  }

  // Prevent the linter from messing up with the beautiful formatting below
  void add_haz_sci_cost
  (// Inputs                                                            // NOLINT
   int haz_it, int sci_it, int nav_cam_start,                           // NOLINT
   double navcam_to_hazcam_timestamp_offset,                            // NOLINT
   double scicam_to_hazcam_timestamp_offset,                            // NOLINT
   MATCH_PAIR                   const & match_pair,                     // NOLINT
   std::vector<double>          const & haz_cam_intensity_timestamps,   // NOLINT
   std::vector<double>          const & ref_timestamps,             // NOLINT
   std::vector<double>          const & sci_cam_timestamps,             // NOLINT
   std::map<int, int>           const & haz_cam_to_left_nav_cam_index,  // NOLINT
   std::map<int, int>           const & haz_cam_to_right_nav_cam_index, // NOLINT
   std::map<int, int>           const & sci_cam_to_left_nav_cam_index,  // NOLINT
   std::map<int, int>           const & sci_cam_to_right_nav_cam_index, // NOLINT
   camera::CameraParameters     const & sci_cam_params,                 // NOLINT
   camera::CameraParameters     const & haz_cam_params,                 // NOLINT
   std::vector<int>             const & depth_to_sci_block_sizes,       // NOLINT
   std::vector<int>             const & depth_to_haz_block_sizes,       // NOLINT
   std::vector<Eigen::Affine3d> const & world_to_ref_t,             // NOLINT
   std::vector<cv::Mat>         const & depth_clouds,                   // NOLINT
   // Outputs                                                           // NOLINT
   std::vector<std::string>           & residual_names,                 // NOLINT
   double                             & hazcam_depth_to_image_scale,    // NOLINT
   std::vector<double>                & world_to_ref_vec,                    // NOLINT
   std::vector<double>                & hazcam_to_navcam_vec,           // NOLINT
   std::vector<double>                & scicam_to_hazcam_vec,           // NOLINT
   std::vector<double>                & hazcam_depth_to_image_vec,      // NOLINT
   Eigen::Vector2d                    & sci_cam_focal_vector,           // NOLINT
   Eigen::Vector2d                    & sci_cam_optical_center,         // NOLINT
   Eigen::VectorXd                    & sci_cam_distortion,             // NOLINT
   ceres::Problem                     & problem) {                      // NOLINT
    auto left_it  = haz_cam_to_left_nav_cam_index.find(haz_it);
    auto right_it = haz_cam_to_right_nav_cam_index.find(haz_it);
    if (left_it == haz_cam_to_left_nav_cam_index.end() ||
        right_it == haz_cam_to_right_nav_cam_index.end())
      LOG(FATAL) << "Book-keeping error 1 in add_haz_sci_cost.";

    int left_nav_it   = left_it->second;
    int right_nav_it  = right_it->second;

    if (nav_cam_start + right_nav_it >= static_cast<int>(ref_timestamps.size()) ||
        haz_it >= static_cast<int>(haz_cam_intensity_timestamps.size()))
      LOG(FATAL) << "Book-keeping error 2 in add_haz_sci_cost.";

    // The haz and sci images must be bracketed by the same two nav images
    {
      auto left_it2  = sci_cam_to_left_nav_cam_index.find(sci_it);
      auto right_it2 = sci_cam_to_right_nav_cam_index.find(sci_it);
      if (left_it2 == sci_cam_to_left_nav_cam_index.end() ||
          right_it2 == sci_cam_to_right_nav_cam_index.end())
        LOG(FATAL) << "Book-keeping error 3 in add_haz_sci_cost.";

      int left_nav_it2   = left_it2->second;
      int right_nav_it2  = right_it2->second;

      if (left_nav_it2 != left_nav_it || right_nav_it2 != right_nav_it)
        LOG(FATAL) << "Book-keeping error 4 in add_haz_sci_cost.";
    }
    // Left and right nav cam image time, in haz_cam's time measurement
    double left_time = ref_timestamps[nav_cam_start + left_nav_it]
      + navcam_to_hazcam_timestamp_offset;
    double right_time = ref_timestamps[nav_cam_start + right_nav_it]
      + navcam_to_hazcam_timestamp_offset;

    // Find the haz and sci time and convert them to haz cam's clock
    double haz_time = haz_cam_intensity_timestamps[haz_it];
    double sci_time = sci_cam_timestamps[sci_it] + scicam_to_hazcam_timestamp_offset;

    bool good1 = (left_time <= haz_time && haz_time < right_time);
    if (!good1) LOG(FATAL) << "Book-keeping error 5 in add_haz_sci_cost.";

    bool good2 = (left_time <= sci_time && sci_time < right_time);
    if (!good2) LOG(FATAL) << "Book-keeping error 6 in add_haz_sci_cost.";

    // Find the transform from the world to nav cam at the haz cam time
    Eigen::Affine3d left_nav_trans = world_to_ref_t[nav_cam_start + left_nav_it];
    Eigen::Affine3d right_nav_trans = world_to_ref_t[nav_cam_start + right_nav_it];

    double alpha_haz = (haz_time - left_time) / (right_time - left_time);
    if (right_time == left_time) alpha_haz = 0.0;  // handle division by zero

    if (!FLAGS_timestamp_interpolation) alpha_haz = round(alpha_haz);

    Eigen::Affine3d world_to_nav_trans_at_haz_time =
      dense_map::linearInterp(alpha_haz, left_nav_trans, right_nav_trans);

    double alpha_sci = (sci_time - left_time) / (right_time - left_time);
    if (right_time == left_time) alpha_sci = 0.0;  // handle division by zero

    if (!FLAGS_timestamp_interpolation)
      alpha_sci = round(alpha_sci);

    Eigen::Affine3d world_to_nav_trans_at_sci_time =
      dense_map::linearInterp(alpha_sci, left_nav_trans, right_nav_trans);

    std::vector<dense_map::InterestPoint> const& haz_ip_vec = match_pair.first;
    std::vector<dense_map::InterestPoint> const& sci_ip_vec = match_pair.second;

    cv::Mat const& depth_cloud = depth_clouds[haz_it];

    for (size_t ip_it = 0; ip_it < haz_ip_vec.size(); ip_it++) {
      // Find the haz cam depth measurement. Use nearest neighbor interpolation
      // to look into the depth cloud.
      int col = round(haz_ip_vec[ip_it].x);
      int row = round(haz_ip_vec[ip_it].y);

      if (col < 0 || row < 0 || col >= depth_cloud.cols || row >= depth_cloud.rows)
        LOG(FATAL) << "Book-keeping error 7 in add_haz_sci_cost.";

      // Skip any point that goes out of bounds due to rounding
      if (col == depth_cloud.cols || row == depth_cloud.rows) continue;

      cv::Vec3f cv_depth_xyz = depth_cloud.at<cv::Vec3f>(row, col);

      // Skip invalid measurements
      if (cv_depth_xyz == cv::Vec3f(0, 0, 0)) continue;

      // Convert to Eigen
      Eigen::Vector3d depth_xyz(cv_depth_xyz[0], cv_depth_xyz[1], cv_depth_xyz[2]);

      // Apply undistortion. Must take great care to not mix up
      // distorted and undistorted pixels.
      Eigen::Vector2d dist_haz_ip(haz_ip_vec[ip_it].x, haz_ip_vec[ip_it].y);
      Eigen::Vector2d dist_sci_ip(sci_ip_vec[ip_it].x, sci_ip_vec[ip_it].y);
      Eigen::Vector2d undist_haz_ip;
      Eigen::Vector2d undist_sci_ip;
      haz_cam_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(dist_haz_ip, &undist_haz_ip);
      sci_cam_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(dist_sci_ip, &undist_sci_ip);

      // Ensure the depth point projects well into the haz cam interest point
      ceres::CostFunction* depth_to_haz_cost_function =
        dense_map::DepthToHazError::Create(undist_haz_ip, depth_xyz, depth_to_haz_block_sizes, haz_cam_params);
      ceres::LossFunction* depth_to_haz_loss_function = dense_map::GetLossFunction("cauchy", FLAGS_robust_threshold);
      problem.AddResidualBlock(depth_to_haz_cost_function, depth_to_haz_loss_function, &hazcam_depth_to_image_vec[0],
                               &hazcam_depth_to_image_scale);
      residual_names.push_back("hazscihaz1");
      residual_names.push_back("hazscihaz2");

      // Ensure that the depth points projects well in the sci cam interest point.
      // Note how we pass a distorted sci cam pix, as in that error function we will
      // take the difference of distorted pixels.
      ceres::CostFunction* depth_to_sci_cost_function
        = dense_map::DepthToSciError::Create(dist_sci_ip, depth_xyz, alpha_haz,
                                             alpha_sci, depth_to_sci_block_sizes, sci_cam_params);
      ceres::LossFunction* depth_to_sci_loss_function
        = dense_map::GetLossFunction("cauchy", FLAGS_robust_threshold);
      problem.AddResidualBlock(depth_to_sci_cost_function, depth_to_sci_loss_function,
                               &world_to_ref_vec[NUM_RIGID_PARAMS * (nav_cam_start + left_nav_it)],
                               &world_to_ref_vec[NUM_RIGID_PARAMS * (nav_cam_start + right_nav_it)],
                               &hazcam_to_navcam_vec[0], &scicam_to_hazcam_vec[0], &hazcam_depth_to_image_vec[0],
                               &hazcam_depth_to_image_scale, &sci_cam_focal_vector[0], &sci_cam_optical_center[0],
                               &sci_cam_distortion[0]);

      residual_names.push_back("hazscisci1");
      residual_names.push_back("hazscisci2");

      if (FLAGS_fix_map) {
        problem.SetParameterBlockConstant(&world_to_ref_vec[NUM_RIGID_PARAMS * (nav_cam_start + left_nav_it)]);
        problem.SetParameterBlockConstant(&world_to_ref_vec[NUM_RIGID_PARAMS * (nav_cam_start + right_nav_it)]);
      }
    }

    return;
  }

  // Prevent the linter from messing up with the beautiful formatting below
  void add_nav_sci_cost
  (// Inputs                                                            // NOLINT
   int nav_it, int sci_it, int nav_cam_start,                           // NOLINT
   double navcam_to_hazcam_timestamp_offset,                            // NOLINT
   double scicam_to_hazcam_timestamp_offset,                            // NOLINT
   MATCH_PAIR                   const & match_pair,                     // NOLINT
   std::vector<double>          const & ref_timestamps,             // NOLINT
   std::vector<double>          const & sci_cam_timestamps,             // NOLINT
   std::map<int, int>           const & sci_cam_to_left_nav_cam_index,  // NOLINT
   std::map<int, int>           const & sci_cam_to_right_nav_cam_index, // NOLINT
   Eigen::Affine3d              const & hazcam_to_navcam_aff_trans,     // NOLINT
   Eigen::Affine3d              const & scicam_to_hazcam_aff_trans,     // NOLINT
   camera::CameraParameters     const & nav_cam_params,                 // NOLINT
   camera::CameraParameters     const & sci_cam_params,                 // NOLINT
   std::vector<int>             const & nav_block_sizes,                // NOLINT
   std::vector<int>             const & sci_block_sizes,                // NOLINT
   std::vector<int>             const & mesh_block_sizes,               // NOLINT
   std::vector<Eigen::Affine3d> const & world_to_ref_t,             // NOLINT
   std::vector<cv::Mat>         const & depth_clouds,                   // NOLINT
   mve::TriangleMesh::Ptr       const & mesh,                           // NOLINT
   std::shared_ptr<BVHTree>     const & bvh_tree,                       // NOLINT
   // Outputs                                                           // NOLINT
   int                                & nav_sci_xyz_count,              // NOLINT
   std::vector<std::string>           & residual_names,                 // NOLINT
   std::vector<double>                & world_to_ref_vec,                    // NOLINT
   std::vector<double>                & hazcam_to_navcam_vec,           // NOLINT
   std::vector<double>                & scicam_to_hazcam_vec,           // NOLINT
   Eigen::Vector2d                    & sci_cam_focal_vector,           // NOLINT
   Eigen::Vector2d                    & sci_cam_optical_center,         // NOLINT
   Eigen::VectorXd                    & sci_cam_distortion,             // NOLINT
   std::vector<Eigen::Vector3d>       & initial_nav_sci_xyz,            // NOLINT
   std::vector<double>                & nav_sci_xyz,                    // NOLINT
   ceres::Problem                     & problem) {                      // NOLINT
    auto left_it  = sci_cam_to_left_nav_cam_index.find(sci_it);
    auto right_it = sci_cam_to_right_nav_cam_index.find(sci_it);
    if (left_it == sci_cam_to_left_nav_cam_index.end() ||
        right_it == sci_cam_to_right_nav_cam_index.end())
      LOG(FATAL) << "Book-keeping error 1 in add_sci_sci_cost.";

    int left_nav_it   = left_it->second;
    int right_nav_it  = right_it->second;

    if (nav_cam_start + right_nav_it >= static_cast<int>(ref_timestamps.size()))
      LOG(FATAL) << "Book-keeping error 1 in add_nav_sci_cost.";

    // Figure out the two nav cam indices bounding the current sci cam
    bool match_left = false;
    if (sci_cam_timestamps[sci_it] + scicam_to_hazcam_timestamp_offset >=
        ref_timestamps[nav_cam_start + nav_it] + navcam_to_hazcam_timestamp_offset) {
      match_left = true;
    } else {
      match_left = false;  // match right then
    }

    // Left and right nav cam image time, and sci cam time, in haz_cam's time measurement
    double left_time = ref_timestamps[nav_cam_start + left_nav_it] + navcam_to_hazcam_timestamp_offset;
    double right_time = ref_timestamps[nav_cam_start + right_nav_it] + navcam_to_hazcam_timestamp_offset;
    double sci_time = sci_cam_timestamps[sci_it] + scicam_to_hazcam_timestamp_offset;

    bool good = (left_time <= sci_time && sci_time < right_time);

    if (!good) LOG(FATAL) << "Book-keeping 2 in add_nav_sci_cost.";

    // Find the transform from the world to nav cam at the sci cam time
    Eigen::Affine3d left_nav_trans = world_to_ref_t[nav_cam_start + left_nav_it];
    Eigen::Affine3d right_nav_trans = world_to_ref_t[nav_cam_start + right_nav_it];
    double alpha = (sci_time - left_time) / (right_time - left_time);
    if (right_time == left_time) alpha = 0.0;  // handle division by zero

    if (!FLAGS_timestamp_interpolation) alpha = round(alpha);

    Eigen::Affine3d interp_world_to_nav_trans = dense_map::linearInterp(alpha, left_nav_trans, right_nav_trans);

    // Find the sci cam to world transform
    Eigen::Affine3d interp_world_to_sci_trans =
      scicam_to_hazcam_aff_trans.inverse() *
      hazcam_to_navcam_aff_trans.inverse() *
      interp_world_to_nav_trans;

    Eigen::Affine3d world_to_nav_trans;
    if (match_left) {
      world_to_nav_trans = left_nav_trans;
    } else {
      world_to_nav_trans = right_nav_trans;
    }

    std::vector<dense_map::InterestPoint> const& sci_ip_vec = match_pair.first;
    std::vector<dense_map::InterestPoint> const& nav_ip_vec = match_pair.second;

    for (size_t ip_it = 0; ip_it < sci_ip_vec.size(); ip_it++) {
      Eigen::Vector2d dist_sci_ip(sci_ip_vec[ip_it].x, sci_ip_vec[ip_it].y);
      Eigen::Vector2d dist_nav_ip(nav_ip_vec[ip_it].x, nav_ip_vec[ip_it].y);
      Eigen::Vector2d undist_nav_ip;
      Eigen::Vector2d undist_sci_ip;
      nav_cam_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(dist_nav_ip, &undist_nav_ip);
      sci_cam_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(dist_sci_ip, &undist_sci_ip);

      Eigen::Vector3d X =
        dense_map::TriangulatePair(sci_cam_params.GetFocalLength(), nav_cam_params.GetFocalLength(),
                                   interp_world_to_sci_trans, world_to_nav_trans, undist_sci_ip, undist_nav_ip);

      bool have_mesh_intersection = false;
      if (FLAGS_mesh != "") {
        // Try to make the intersection point be on the mesh and the nav cam ray
        // to make the sci cam to conform to that.
        // TODO(oalexan1): Think more of the range of the ray below
        double min_ray_dist = 0.0;
        double max_ray_dist = 10.0;
        Eigen::Vector3d intersection(0.0, 0.0, 0.0);
        have_mesh_intersection
          = dense_map::ray_mesh_intersect(undist_nav_ip, nav_cam_params,
                                          world_to_nav_trans, mesh, bvh_tree,
                                          min_ray_dist, max_ray_dist,
                                          // Output
                                          intersection);

        // Overwrite the triangulation result above with the intersection
        if (have_mesh_intersection) X = intersection;
      }

      // Record the triangulated positions. These will be optimized.
      for (int i = 0; i < dense_map::NUM_XYZ_PARAMS; i++)
        nav_sci_xyz[dense_map::NUM_XYZ_PARAMS * nav_sci_xyz_count + i] = X[i];

      // A copy of the triangulated positions which won't be optimized.
      initial_nav_sci_xyz[nav_sci_xyz_count] = X;

      // The cost function of projecting in the sci cam. Note that we use dist_sci_ip,
      // as in the cost function below we will do differences of distorted sci cam pixels.
      ceres::CostFunction* sci_cost_function =
        dense_map::SciError::Create(dist_sci_ip, alpha, sci_block_sizes, sci_cam_params);
      ceres::LossFunction* sci_loss_function = dense_map::GetLossFunction("cauchy", FLAGS_robust_threshold);

      problem.AddResidualBlock(sci_cost_function, sci_loss_function,
                               &world_to_ref_vec[NUM_RIGID_PARAMS * (nav_cam_start + left_nav_it)],
                               &world_to_ref_vec[NUM_RIGID_PARAMS * (nav_cam_start + right_nav_it)],
                               &hazcam_to_navcam_vec[0], &scicam_to_hazcam_vec[0],
                               &nav_sci_xyz[dense_map::NUM_XYZ_PARAMS * nav_sci_xyz_count], &sci_cam_focal_vector[0],
                               &sci_cam_optical_center[0], &sci_cam_distortion[0]);

      residual_names.push_back("navscisci1");
      residual_names.push_back("navscisci2");

      if (FLAGS_fix_map) {
        problem.SetParameterBlockConstant(&world_to_ref_vec[NUM_RIGID_PARAMS * (nav_cam_start + left_nav_it)]);
        problem.SetParameterBlockConstant(&world_to_ref_vec[NUM_RIGID_PARAMS * (nav_cam_start + right_nav_it)]);
      }

      // The nav cam cost function
      ceres::CostFunction* nav_cost_function =
        dense_map::NavError::Create(undist_nav_ip, nav_block_sizes, nav_cam_params);
      ceres::LossFunction* nav_loss_function =
        dense_map::GetLossFunction("cauchy", FLAGS_robust_threshold);

      problem.AddResidualBlock(nav_cost_function, nav_loss_function,
                               &world_to_ref_vec[(nav_cam_start + nav_it) * NUM_RIGID_PARAMS],
                               &nav_sci_xyz[dense_map::NUM_XYZ_PARAMS * nav_sci_xyz_count]);
      residual_names.push_back("navscinav1");
      residual_names.push_back("navscinav2");

      if (FLAGS_fix_map)
        problem.SetParameterBlockConstant(&world_to_ref_vec[(nav_cam_start + nav_it)
                                                       * NUM_RIGID_PARAMS]);

      if (have_mesh_intersection) {
        // Constrain the sci cam texture to agree with the nav cam texture on the mesh

        ceres::CostFunction* mesh_cost_function =
          dense_map::XYZError::Create(initial_nav_sci_xyz[nav_sci_xyz_count],
                                      mesh_block_sizes, FLAGS_mesh_weight);

        ceres::LossFunction* mesh_loss_function =
          dense_map::GetLossFunction("cauchy", FLAGS_mesh_robust_threshold);

        problem.AddResidualBlock(mesh_cost_function, mesh_loss_function,
                                 &nav_sci_xyz[dense_map::NUM_XYZ_PARAMS * nav_sci_xyz_count]);

        residual_names.push_back("mesh_x");
        residual_names.push_back("mesh_y");
        residual_names.push_back("mesh_z");
      }

      // Very important, go forward in the vector of xyz. Do it at the end.
      nav_sci_xyz_count++;
    }

    return;
  }

  void adjustImageSize(camera::CameraParameters const& cam_params, cv::Mat & image) {
    int raw_image_cols = image.cols;
    int raw_image_rows = image.rows;
    int calib_image_cols = cam_params.GetDistortedSize()[0];
    int calib_image_rows = cam_params.GetDistortedSize()[1];

    int factor = raw_image_cols / calib_image_cols;

    if ((raw_image_cols != calib_image_cols * factor) || (raw_image_rows != calib_image_rows * factor)) {
      LOG(FATAL) << "Image width and height are: " << raw_image_cols << ' ' << raw_image_rows << "\n"
                 << "Calibrated image width and height are: "
                 << calib_image_cols << ' ' << calib_image_rows << "\n"
                 << "These must be equal up to an integer factor.\n";
    }

    if (factor != 1) {
      // TODO(oalexan1): This kind of resizing may be creating aliased images.
      cv::Mat local_image;
      cv::resize(image, local_image, cv::Size(), 1.0/factor, 1.0/factor, cv::INTER_AREA);
      local_image.copyTo(image);
    }

    // Check
    if (image.cols != calib_image_cols || image.rows != calib_image_rows)
      LOG(FATAL) << "Sci cam images have the wrong size.";
  }

  void select_images_to_match(// Inputs                                                // NOLINT
                              double haz_cam_start_time,                               // NOLINT
                              double navcam_to_hazcam_timestamp_offset,                // NOLINT
                              double scicam_to_hazcam_timestamp_offset,                // NOLINT
                              std::vector<double> const& ref_timestamps,        // NOLINT
                              std::vector<double> const& all_haz_cam_inten_timestamps, // NOLINT
                              std::vector<double> const& all_sci_cam_timestamps,       // NOLINT
                              std::set<double> const& sci_cam_timestamps_to_use,       // NOLINT
                              dense_map::RosBagHandle const& nav_cam_handle,           // NOLINT
                              dense_map::RosBagHandle const& sci_cam_handle,           // NOLINT
                              dense_map::RosBagHandle const& haz_cam_points_handle,    // NOLINT
                              dense_map::RosBagHandle const& haz_cam_intensity_handle, // NOLINT
                              camera::CameraParameters const& sci_cam_params,          // NOLINT
                              // Outputs                                               // NOLINT
                              int& nav_cam_start,                                      // NOLINT
                              std::vector<imgType>& cid_to_image_type,                 // NOLINT
                              std::vector<double>& haz_cam_intensity_timestamps,       // NOLINT
                              std::vector<double>& sci_cam_timestamps,                 // NOLINT
                              std::vector<cv::Mat>& images,                            // NOLINT
                              std::vector<cv::Mat>& depth_clouds) {                    // NOLINT
    // Wipe the outputs
    nav_cam_start = -1;
    cid_to_image_type.clear();
    haz_cam_intensity_timestamps.clear();
    sci_cam_timestamps.clear();
    images.clear();
    depth_clouds.clear();

    bool stop_early = false;
    double found_time = -1.0;
    bool save_grayscale = true;  // feature detection needs grayscale

    double navcam_to_scicam_timestamp_offset
      = navcam_to_hazcam_timestamp_offset - scicam_to_hazcam_timestamp_offset;

    // Use these to keep track where in the bags we are. After one
    // traversal forward in time they need to be reset.
    int nav_cam_pos = 0, haz_cam_intensity_pos = 0, haz_cam_cloud_pos = 0, sci_cam_pos = 0;

    for (size_t map_it = 0; map_it + 1 < ref_timestamps.size(); map_it++) {
      if (FLAGS_start >= 0.0 && FLAGS_duration > 0.0) {
        // The case when we would like to start later. Note the second
        // comparison after "&&".  When FLAG_start is 0, we want to
        // make sure if the first nav image from the bag is in the map
        // we use it, so we don't skip it even if based on
        // navcam_to_hazcam_timestamp_offset we should.
        if (ref_timestamps[map_it] + navcam_to_hazcam_timestamp_offset
            < FLAGS_start + haz_cam_start_time &&
            ref_timestamps[map_it] < FLAGS_start + haz_cam_start_time)
          continue;
      }

      if (nav_cam_start < 0) nav_cam_start = map_it;

      images.push_back(cv::Mat());
      if (!dense_map::lookupImage(ref_timestamps[map_it], nav_cam_handle.bag_msgs,
                                  save_grayscale, images.back(),
                                  nav_cam_pos, found_time)) {
        LOG(FATAL) << std::fixed << std::setprecision(17)
                   << "Cannot look up nav cam at time " << ref_timestamps[map_it] << ".\n";
      }
      cid_to_image_type.push_back(dense_map::NAV_CAM);

      if (FLAGS_start >= 0.0 && FLAGS_duration > 0.0) {
        // If we would like to end earlier, then save the last nav cam image so far
        // and quit
        if (ref_timestamps[map_it] + navcam_to_hazcam_timestamp_offset >
            FLAGS_start + FLAGS_duration + haz_cam_start_time) {
          stop_early = true;
          break;
        }
      }

      // Do not look up sci cam and haz cam images in time intervals bigger than this
      if (std::abs(ref_timestamps[map_it + 1] - ref_timestamps[map_it]) > FLAGS_bracket_len) continue;

      // Append at most two haz cam images between consecutive sparse
      // map timestamps, close to these sparse map timestamps.
      std::vector<double> local_haz_timestamps;
      dense_map::pickTimestampsInBounds(all_haz_cam_inten_timestamps,
                                        ref_timestamps[map_it],
                                        ref_timestamps[map_it + 1],
                                        -navcam_to_hazcam_timestamp_offset,
                                        local_haz_timestamps);

      for (size_t samp_it = 0; samp_it < local_haz_timestamps.size(); samp_it++) {
        haz_cam_intensity_timestamps.push_back(local_haz_timestamps[samp_it]);

        double nav_start = ref_timestamps[map_it] + navcam_to_hazcam_timestamp_offset
          - haz_cam_start_time;
        double haz_time = local_haz_timestamps[samp_it] - haz_cam_start_time;
        double nav_end =  ref_timestamps[map_it + 1] + navcam_to_hazcam_timestamp_offset
          - haz_cam_start_time;

        std::cout << "nav_start haz nav_end times "
                  << nav_start << ' ' << haz_time << ' ' << nav_end  << std::endl;
        std::cout << "xxxhaz before " << haz_time - nav_start << ' ' << nav_end - haz_time << ' ' << nav_end - nav_start
                  << std::endl;
        std::cout << "nav_end - nav_start " << nav_end - nav_start << std::endl;

        // Read the image
        images.push_back(cv::Mat());
        if (!dense_map::lookupImage(haz_cam_intensity_timestamps.back(),
                                    haz_cam_intensity_handle.bag_msgs,
                                    save_grayscale, images.back(), haz_cam_intensity_pos,
                                    found_time))
          LOG(FATAL) << "Cannot look up haz cam image at given time";
        cid_to_image_type.push_back(dense_map::HAZ_CAM);

        double cloud_time = -1.0;
        depth_clouds.push_back(cv::Mat());
        if (!dense_map::lookupCloud(haz_cam_intensity_timestamps.back(),
                                    haz_cam_points_handle.bag_msgs,
                                    FLAGS_max_haz_cam_image_to_depth_timestamp_diff,
                                    depth_clouds.back(),
                                    haz_cam_cloud_pos, cloud_time)) {
          // This need not succeed always
        }
      }

      // Append at most two sci cam images between consecutive sparse
      // map timestamps, close to these sparse map timestamps.

      std::vector<double> local_sci_timestamps;
      dense_map::pickTimestampsInBounds(all_sci_cam_timestamps, ref_timestamps[map_it],
                                        ref_timestamps[map_it + 1],
                                        -navcam_to_scicam_timestamp_offset,
                                        local_sci_timestamps);

      // Append to the vector of sampled timestamps
      for (size_t samp_it = 0; samp_it < local_sci_timestamps.size(); samp_it++) {
        // See if to use only specified timestamps
        if (!sci_cam_timestamps_to_use.empty() &&
            sci_cam_timestamps_to_use.find(local_sci_timestamps[samp_it]) ==
            sci_cam_timestamps_to_use.end())
          continue;

        sci_cam_timestamps.push_back(local_sci_timestamps[samp_it]);

        double nav_start = ref_timestamps[map_it] + navcam_to_hazcam_timestamp_offset
          - haz_cam_start_time;
        double sci_time = local_sci_timestamps[samp_it] + scicam_to_hazcam_timestamp_offset
          - haz_cam_start_time;
        double nav_end = ref_timestamps[map_it + 1] + navcam_to_hazcam_timestamp_offset
          - haz_cam_start_time;
        std::cout << "nav_start sci nav_end times "
                  << nav_start << ' ' << sci_time << ' ' << nav_end  << std::endl;
        std::cout << "xxxsci before " << sci_time - nav_start << ' ' << nav_end - sci_time << ' ' << nav_end - nav_start
                  << std::endl;

        std::cout << "nav_end - nav_start " << nav_end - nav_start << std::endl;

        // Read the sci cam image, and perhaps adjust its size
        images.push_back(cv::Mat());
        cv::Mat local_img;
        if (!dense_map::lookupImage(sci_cam_timestamps.back(), sci_cam_handle.bag_msgs,
                                    save_grayscale, local_img,
                                    sci_cam_pos, found_time))
          LOG(FATAL) << "Cannot look up sci cam image at given time.";
        adjustImageSize(sci_cam_params, local_img);
        local_img.copyTo(images.back());

        // Sanity check
        Eigen::Vector2i sci_cam_size = sci_cam_params.GetDistortedSize();
        if (images.back().cols != sci_cam_size[0] || images.back().rows != sci_cam_size[1])
            LOG(FATAL) << "Sci cam images have the wrong size.";

        cid_to_image_type.push_back(dense_map::SCI_CAM);
      }
    }  // End iterating over nav cam timestamps

    // Add the last nav cam image from the map, unless we stopped early and this was done
    if (!stop_early) {
      images.push_back(cv::Mat());
      if (!dense_map::lookupImage(ref_timestamps.back(), nav_cam_handle.bag_msgs,
                                  save_grayscale, images.back(),
                                  nav_cam_pos, found_time))
        LOG(FATAL) << "Cannot look up nav cam image at given time.";
      cid_to_image_type.push_back(dense_map::NAV_CAM);
    }

    if (images.size() > ref_timestamps.size() + haz_cam_intensity_timestamps.size()
        + sci_cam_timestamps.size())
      LOG(FATAL) << "Book-keeping error in select_images_to_match.";

    return;
  }

  void set_up_block_sizes(int num_scicam_focal_lengths, int num_scicam_distortions,
                          std::vector<int> & depth_to_haz_block_sizes,
                          std::vector<int> & nav_block_sizes,
                          std::vector<int> & depth_to_nav_block_sizes,
                          std::vector<int> & depth_to_sci_block_sizes,
                          std::vector<int> & sci_block_sizes,
                          std::vector<int> & mesh_block_sizes) {
    // Wipe the outputs
    depth_to_haz_block_sizes.clear();
    nav_block_sizes.clear();
    depth_to_nav_block_sizes.clear();
    depth_to_sci_block_sizes.clear();
    sci_block_sizes.clear();
    mesh_block_sizes.clear();

    // Set up the variable blocks to optimize for DepthToHazError
    depth_to_haz_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    depth_to_haz_block_sizes.push_back(dense_map::NUM_SCALAR_PARAMS);

    // Set up the variable blocks to optimize for NavError
    nav_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    nav_block_sizes.push_back(dense_map::NUM_XYZ_PARAMS);

    // Set up the variable blocks to optimize for DepthToNavError
    depth_to_nav_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    depth_to_nav_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    depth_to_nav_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    depth_to_nav_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    depth_to_nav_block_sizes.push_back(dense_map::NUM_SCALAR_PARAMS);

    // Set up the variable blocks to optimize for DepthToSciError
    depth_to_sci_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    depth_to_sci_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    depth_to_sci_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    depth_to_sci_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    depth_to_sci_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    depth_to_sci_block_sizes.push_back(dense_map::NUM_SCALAR_PARAMS);
    depth_to_sci_block_sizes.push_back(num_scicam_focal_lengths);       // focal length
    depth_to_sci_block_sizes.push_back(dense_map::NUM_OPT_CTR_PARAMS);  // optical center
    depth_to_sci_block_sizes.push_back(num_scicam_distortions);         // distortion

    // Set up the variable blocks to optimize for SciError
    sci_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    sci_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    sci_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    sci_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
    sci_block_sizes.push_back(dense_map::NUM_XYZ_PARAMS);
    sci_block_sizes.push_back(num_scicam_focal_lengths);       // focal length
    sci_block_sizes.push_back(dense_map::NUM_OPT_CTR_PARAMS);  // optical center
    sci_block_sizes.push_back(num_scicam_distortions);         // distortion

    // Set up the variable blocks to optimize for the mesh xyz error
    mesh_block_sizes.push_back(dense_map::NUM_XYZ_PARAMS);
  }

  typedef std::map<std::pair<int, int>, dense_map::MATCH_PAIR> MATCH_MAP;

  // Wrapper to find the value of a const map at a given key
  int mapVal(std::map<int, int> const& map, int key) {
    auto ptr = map.find(key);
    if (ptr == map.end())
      LOG(FATAL) << "Cannot find map value for given index.";
    return ptr->second;
  }

  // Find nav images close in time. Match sci and haz images in between to each
  // other and to these nave images
  // TODO(oalexan1): Reword the above explanation
  void detect_match_features(// Inputs                                            // NOLINT
                             std::vector<cv::Mat> const& images,                  // NOLINT
                             std::vector<imgType> const& cid_to_image_type,       // NOLINT
                             // Outputs                                           // NOLINT
                             std::map<int, int> & image_to_nav_it,                // NOLINT
                             std::map<int, int> & image_to_haz_it,                // NOLINT
                             std::map<int, int> & image_to_sci_it,                // NOLINT
                             std::map<int, int> & haz_cam_to_left_nav_cam_index,  // NOLINT
                             std::map<int, int> & haz_cam_to_right_nav_cam_index, // NOLINT
                             std::map<int, int> & sci_cam_to_left_nav_cam_index,  // NOLINT
                             std::map<int, int> & sci_cam_to_right_nav_cam_index, // NOLINT
                             MATCH_MAP          & matches) {                      // NOLINT
    // Wipe the outputs
    image_to_nav_it.clear();
    image_to_haz_it.clear();
    image_to_sci_it.clear();
    haz_cam_to_left_nav_cam_index.clear();
    haz_cam_to_right_nav_cam_index.clear();
    sci_cam_to_left_nav_cam_index.clear();
    sci_cam_to_right_nav_cam_index.clear();
    matches.clear();

    int nav_it = 0, haz_it = 0, sci_it = 0;
    for (int image_it = 0; image_it < static_cast<int>(images.size()); image_it++) {
      if (cid_to_image_type[image_it] == dense_map::NAV_CAM) {
        image_to_nav_it[image_it] = nav_it;
        nav_it++;
      } else if (cid_to_image_type[image_it] == dense_map::HAZ_CAM) {
        image_to_haz_it[image_it] = haz_it;
        haz_it++;
      } else if (cid_to_image_type[image_it] == dense_map::SCI_CAM) {
        image_to_sci_it[image_it] = sci_it;
        sci_it++;
      }
    }

    std::vector<std::pair<int, int> > image_pairs;

    // Look at two neighboring nav images. Find left_img_it and
    // right_img_it so cid_to_image_type for these are nav images.
    for (int left_img_it = 0; left_img_it < static_cast<int>(images.size()); left_img_it++) {
      if (cid_to_image_type[left_img_it] != dense_map::NAV_CAM) continue;  // Not nav cam

      // now find right_img_it
      int right_img_it = -1;
      for (int local_it = left_img_it + 1; local_it < static_cast<int>(images.size()); local_it++) {
        if (cid_to_image_type[local_it] == dense_map::NAV_CAM) {
          right_img_it = local_it;
          break;
        }
      }

      if (right_img_it < 0) continue;

      // Now look at sci and haz images in between
      std::vector<int> nav_cam_indices, haz_cam_indices, sci_cam_indices;

      nav_cam_indices.push_back(left_img_it);
      nav_cam_indices.push_back(right_img_it);

      for (int local_img_it = left_img_it + 1; local_img_it < right_img_it; local_img_it++) {
        int left_nav_it  = mapVal(image_to_nav_it, left_img_it);
        int right_nav_it = mapVal(image_to_nav_it, right_img_it);

        if (cid_to_image_type[local_img_it] == dense_map::HAZ_CAM) {
          int haz_it = mapVal(image_to_haz_it, local_img_it);
          haz_cam_indices.push_back(local_img_it);
          haz_cam_to_left_nav_cam_index[haz_it]  = left_nav_it;
          haz_cam_to_right_nav_cam_index[haz_it] = right_nav_it;

        } else if (cid_to_image_type[local_img_it] == dense_map::SCI_CAM) {
          int sci_it = mapVal(image_to_sci_it, local_img_it);
          sci_cam_indices.push_back(local_img_it);
          sci_cam_to_left_nav_cam_index[image_to_sci_it[local_img_it]] = left_nav_it;
          sci_cam_to_right_nav_cam_index[image_to_sci_it[local_img_it]] = right_nav_it;
        }
      }

      // Match haz to nav
      for (size_t haz_it = 0; haz_it < haz_cam_indices.size(); haz_it++) {
        for (size_t nav_it = 0; nav_it < nav_cam_indices.size(); nav_it++) {
          image_pairs.push_back(std::make_pair(haz_cam_indices[haz_it], nav_cam_indices[nav_it]));
        }
      }

      // Match haz to sci
      for (size_t haz_it = 0; haz_it < haz_cam_indices.size(); haz_it++) {
        for (size_t sci_it = 0; sci_it < sci_cam_indices.size(); sci_it++) {
          image_pairs.push_back(std::make_pair(haz_cam_indices[haz_it], sci_cam_indices[sci_it]));
        }
      }

      // Match sci to nav
      for (size_t nav_it = 0; nav_it < nav_cam_indices.size(); nav_it++) {
        for (size_t sci_it = 0; sci_it < sci_cam_indices.size(); sci_it++) {
          image_pairs.push_back(std::make_pair(sci_cam_indices[sci_it], nav_cam_indices[nav_it]));
        }
      }
    }

    // Detect features using multiple threads
    std::vector<cv::Mat> cid_to_descriptor_map;
    std::vector<Eigen::Matrix2Xd> cid_to_keypoint_map;
    cid_to_descriptor_map.resize(images.size());
    cid_to_keypoint_map.resize(images.size());
    ff_common::ThreadPool thread_pool1;
    for (size_t it = 0; it < images.size(); it++)
      thread_pool1.AddTask(&dense_map::detectFeatures, images[it], FLAGS_verbose,
                           &cid_to_descriptor_map[it], &cid_to_keypoint_map[it]);
    thread_pool1.Join();

    // Create the matches among nav, haz, and sci images. Note that we
    // have a starting and ending nav cam images, and match all the
    // haz and sci images to these two nav cam images and to each
    // other.

    // Find the matches using multiple threads
    ff_common::ThreadPool thread_pool2;
    std::mutex match_mutex;
    for (size_t pair_it = 0; pair_it < image_pairs.size(); pair_it++) {
      auto pair = image_pairs[pair_it];
      int left_image_it = pair.first, right_image_it = pair.second;
      thread_pool2.AddTask(&dense_map::matchFeatures, &match_mutex,
                           left_image_it, right_image_it,
                           cid_to_descriptor_map[left_image_it],
                           cid_to_descriptor_map[right_image_it],
                           cid_to_keypoint_map[left_image_it],
                           cid_to_keypoint_map[right_image_it],
                           FLAGS_verbose, &matches[pair]);
    }
    thread_pool2.Join();

    return;
  }

  // A class to encompass all known information about a camera
  // This is work in progress and will replace some of the logic further down.
  struct cameraImage {
    // An index to look up the type of camera. This will equal the
    // value ref_camera_type if and only if this is a reference
    // camera.
    int camera_type;

    // The timestamp for this camera (in floating point seconds since epoch)
    double timestamp;

    // The timestamp with an adjustment added to it to be in
    // reference camera time
    double ref_timestamp;

    // Indices to look up the reference cameras bracketing this camera
    // in time. The two indices will have same value if and only if
    // this is a reference camera.
    int beg_ref_index;
    int end_ref_index;

    // The image for this camera, in grayscale
    cv::Mat image;

    // The corresponding depth cloud, for an image + depth camera
    cv::Mat depth_cloud;
  };

  // Sort by timestamps in the ref camera clock
  bool timestampLess(cameraImage i, cameraImage j) {
    return (i.ref_timestamp < j.ref_timestamp);
  }

  // Find the haz cam depth measurement. Use nearest neighbor interpolation
  // to look into the depth cloud.
  bool depthValue(  // Inputs
    cv::Mat const& depth_cloud, Eigen::Vector2d const& dist_ip,
    // Output
    Eigen::Vector3d& depth_xyz) {
    depth_xyz = Eigen::Vector3d(0, 0, 0);  // initialize

    if (depth_cloud.cols == 0 && depth_cloud.rows == 0) return false;  // empty cloud

    int col = round(dist_ip[0]);
    int row = round(dist_ip[1]);

    if (col < 0 || row < 0 || col > depth_cloud.cols || row > depth_cloud.rows)
      LOG(FATAL) << "Out of range in depth cloud.";

    // After rounding one may hit the bound
    if (col == depth_cloud.cols || row == depth_cloud.rows)
      return false;

    cv::Vec3f cv_depth_xyz = depth_cloud.at<cv::Vec3f>(row, col);

    // Skip invalid measurements
    if (cv_depth_xyz == cv::Vec3f(0, 0, 0))
      return false;

    depth_xyz = Eigen::Vector3d(cv_depth_xyz[0], cv_depth_xyz[1], cv_depth_xyz[2]);

    return true;
  }

}  // namespace dense_map

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  std::cout << "--now in main!" << std::endl;
  std::cout << "--start and duration not supported!" << std::endl;
  std::cout.precision(17);  // to be able to print timestamps

  if (FLAGS_ros_bag.empty())
    LOG(FATAL) << "The bag file was not specified.";
  if (FLAGS_sparse_map.empty())
    LOG(FATAL) << "The input sparse map was not specified.";

  if (FLAGS_output_map.empty())
    LOG(FATAL) << "The output sparse map was not specified.";

  if (!FLAGS_skip_registration) {
    if (FLAGS_xyz_file.empty() || FLAGS_hugin_file.empty())
      LOG(FATAL) << "Either the hugin or xyz file was not specified.";
  }

  if (FLAGS_opt_map_only && FLAGS_fix_map)
    LOG(FATAL) << "Cannot both float the sparse map and keep it fixed.";

  if (FLAGS_robust_threshold <= 0.0)
    LOG(FATAL) << "The robust threshold must be positive.\n";

  // Set up handles for reading data at given time stamp without
  // searching through the whole bag each time.
  dense_map::RosBagHandle nav_cam_handle(FLAGS_ros_bag, FLAGS_nav_cam_topic);
  dense_map::RosBagHandle sci_cam_handle(FLAGS_ros_bag, FLAGS_sci_cam_topic);
  dense_map::RosBagHandle haz_cam_points_handle(FLAGS_ros_bag, FLAGS_haz_cam_points_topic);
  dense_map::RosBagHandle haz_cam_intensity_handle(FLAGS_ros_bag, FLAGS_haz_cam_intensity_topic);

  if (nav_cam_handle.bag_msgs.empty()) LOG(FATAL) << "No nav cam images found.";
  if (sci_cam_handle.bag_msgs.empty()) LOG(FATAL) << "No sci cam images found.";
  if (haz_cam_intensity_handle.bag_msgs.empty()) LOG(FATAL) << "No haz cam images found.";

  // Read the config file
  double navcam_to_hazcam_timestamp_offset = 0.0, scicam_to_hazcam_timestamp_offset = 0.0;
  Eigen::MatrixXd hazcam_to_navcam_trans = Eigen::MatrixXd::Identity(4, 4);
  Eigen::MatrixXd scicam_to_hazcam_trans = Eigen::MatrixXd::Identity(4, 4);
  Eigen::MatrixXd navcam_to_navcam_trans = Eigen::MatrixXd::Identity(4, 4);
  Eigen::MatrixXd navcam_to_body_trans = Eigen::MatrixXd::Identity(4, 4);
  Eigen::Affine3d hazcam_depth_to_image_transform;
  hazcam_depth_to_image_transform.setIdentity();  // default value
  camera::CameraParameters nav_cam_params(Eigen::Vector2i(0, 0), Eigen::Vector2d(0, 0),
                                          Eigen::Vector2d(0, 0));
  camera::CameraParameters haz_cam_params(Eigen::Vector2i(0, 0), Eigen::Vector2d(0, 0),
                                          Eigen::Vector2d(0, 0));
  camera::CameraParameters sci_cam_params(Eigen::Vector2i(0, 0), Eigen::Vector2d(0, 0),
                                          Eigen::Vector2d(0, 0));
  dense_map::readConfigFile("navcam_to_hazcam_timestamp_offset", // NOLINT
                            "scicam_to_hazcam_timestamp_offset", // NOLINT
                            "hazcam_to_navcam_transform",        // NOLINT
                            "scicam_to_hazcam_transform",        // NOLINT
                            "nav_cam_transform",                 // NOLINT
                            "hazcam_depth_to_image_transform",   // NOLINT
                            navcam_to_hazcam_timestamp_offset,
                            scicam_to_hazcam_timestamp_offset,
                            hazcam_to_navcam_trans, scicam_to_hazcam_trans,
                            navcam_to_body_trans, hazcam_depth_to_image_transform,
                            nav_cam_params, haz_cam_params,
                            sci_cam_params);

  if (!std::isnan(FLAGS_scicam_to_hazcam_timestamp_offset_override_value)) {
    double new_val = FLAGS_scicam_to_hazcam_timestamp_offset_override_value;
    std::cout << "Overriding the value " << scicam_to_hazcam_timestamp_offset
              << " of scicam_to_hazcam_timestamp_offset with: " << new_val << std::endl;
    scicam_to_hazcam_timestamp_offset = new_val;
  }

  if (FLAGS_mesh_weight <= 0.0 || FLAGS_mesh_robust_threshold <= 0.0)
    LOG(FATAL) << "The mesh weight and robust threshold must be positive.\n";

  mve::TriangleMesh::Ptr mesh;
  std::shared_ptr<mve::MeshInfo> mesh_info;
  std::shared_ptr<tex::Graph> graph;
  std::shared_ptr<BVHTree> bvh_tree;
  if (FLAGS_mesh != "") dense_map::loadMeshBuildTree(FLAGS_mesh, mesh, mesh_info, graph, bvh_tree);

#if 0
  std::cout << "hazcam_to_navcam_trans\n" << hazcam_to_navcam_trans << std::endl;
  std::cout << "scicam_to_hazcam_trans\n" << scicam_to_hazcam_trans << std::endl;
  std::cout << "navcam_to_hazcam_timestamp_offset: " << navcam_to_hazcam_timestamp_offset << "\n";
  std::cout << "scicam_to_hazcam_timestamp_offset: " << scicam_to_hazcam_timestamp_offset << "\n";
  std::cout << "hazcam_depth_to_image_transform\n"   << hazcam_depth_to_image_transform.matrix()
            << "\n";
#endif

  double hazcam_depth_to_image_scale
    = pow(hazcam_depth_to_image_transform.matrix().determinant(), 1.0 / 3.0);

  // Since we will keep the scale fixed, vary the part of the transform without
  // the scale, while adding the scale each time before the transform is applied
  Eigen::Affine3d hazcam_depth_to_image_noscale = hazcam_depth_to_image_transform;
  hazcam_depth_to_image_noscale.linear() /= hazcam_depth_to_image_scale;

  // Convert hazcam_to_navcam_trans to Affine3d
  Eigen::Affine3d hazcam_to_navcam_aff_trans;
  hazcam_to_navcam_aff_trans.matrix() = hazcam_to_navcam_trans;

  // Convert scicam_to_hazcam_trans to Affine3d
  Eigen::Affine3d scicam_to_hazcam_aff_trans;
  scicam_to_hazcam_aff_trans.matrix() = scicam_to_hazcam_trans;

  // Read the sparse map
  boost::shared_ptr<sparse_mapping::SparseMap> sparse_map =
    boost::shared_ptr<sparse_mapping::SparseMap>(new sparse_mapping::SparseMap(FLAGS_sparse_map));

  // TODO(oalexan1): All this timestamp reading logic below should be in a function

  // Find the minimum and maximum timestamps in the sparse map
  double min_map_timestamp = std::numeric_limits<double>::max();
  double max_map_timestamp = -min_map_timestamp;
  std::vector<double> ref_timestamps;
  const std::vector<std::string>& sparse_map_images = sparse_map->cid_to_filename_;
  ref_timestamps.resize(sparse_map_images.size());
  for (size_t cid = 0; cid < sparse_map_images.size(); cid++) {
    double timestamp = dense_map::fileNameToTimestamp(sparse_map_images[cid]);
    ref_timestamps[cid] = timestamp;
    min_map_timestamp = std::min(min_map_timestamp, ref_timestamps[cid]);
    max_map_timestamp = std::max(max_map_timestamp, ref_timestamps[cid]);
  }
  if (ref_timestamps.empty()) LOG(FATAL) << "No sparse map timestamps found.";

  // Read the haz cam timestamps
  std::vector<rosbag::MessageInstance> const& haz_cam_intensity_msgs
    = haz_cam_intensity_handle.bag_msgs;
  if (haz_cam_intensity_msgs.empty()) LOG(FATAL) << "No haz cam messages are present.";
  double haz_cam_start_time = -1.0;
  std::vector<double> all_haz_cam_inten_timestamps;
  for (size_t it = 0; it < haz_cam_intensity_msgs.size(); it++) {
    sensor_msgs::Image::ConstPtr image_msg
      = haz_cam_intensity_msgs[it].instantiate<sensor_msgs::Image>();
    if (image_msg) {
      double haz_cam_time = image_msg->header.stamp.toSec();
      all_haz_cam_inten_timestamps.push_back(haz_cam_time);
      if (haz_cam_start_time < 0) haz_cam_start_time = haz_cam_time;
    }
  }

  // Read the sci cam timestamps from the bag
  std::vector<rosbag::MessageInstance> const& sci_cam_msgs = sci_cam_handle.bag_msgs;
  std::vector<double> all_sci_cam_timestamps;
  for (size_t sci_it = 0; sci_it < sci_cam_msgs.size(); sci_it++) {
    sensor_msgs::Image::ConstPtr sci_image_msg = sci_cam_msgs[sci_it].instantiate<sensor_msgs::Image>();
    sensor_msgs::CompressedImage::ConstPtr comp_sci_image_msg =
      sci_cam_msgs[sci_it].instantiate<sensor_msgs::CompressedImage>();
    if (sci_image_msg)
      all_sci_cam_timestamps.push_back(sci_image_msg->header.stamp.toSec());
    else if (comp_sci_image_msg)
      all_sci_cam_timestamps.push_back(comp_sci_image_msg->header.stamp.toSec());
  }

  // If desired to process only specific timestamps
  std::set<double> sci_cam_timestamps_to_use;
  if (FLAGS_sci_cam_timestamps != "") {
    std::ifstream ifs(FLAGS_sci_cam_timestamps.c_str());
    double val;
    while (ifs >> val) sci_cam_timestamps_to_use.insert(val);
  }

  // Will optimize the nav cam poses as part of the process
  std::vector<Eigen::Affine3d>& world_to_ref_t = sparse_map->cid_to_cam_t_global_;  // alias

  // Put transforms of the reference cameras in a vector. We will optimize them.
  int num_ref_cams = world_to_ref_t.size();
  if (world_to_ref_t.size() != ref_timestamps.size())
    LOG(FATAL) << "Must have as many ref cam timestamps as ref cameras.\n";
  std::vector<double> world_to_ref_vec(num_ref_cams * dense_map::NUM_RIGID_PARAMS);
  for (int cid = 0; cid < num_ref_cams; cid++)
    dense_map::rigid_transform_to_array(world_to_ref_t[cid],
                                        &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * cid]);


  // We assume our camera rig has n camera types. Each can be image or
  // depth + image.  Just one camera must be the reference camera. In
  // this code it will be nav_cam.  Every camera object (class
  // cameraImage) knows its type (an index), which can be used to look
  // up its intrinsics, image topic, depth topic (if present),
  // ref_to_cam_timestamp_offset, and ref_to_cam_transform.  A camera
  // object also stores its image, depth cloud (if present), its
  // timestamp, and indices pointing to its left and right ref
  // bracketing cameras.

  // For every instance of a reference camera its
  // ref_to_cam_timestamp_offset is 0 and kept fixed,
  // ref_to_cam_transform is the identity and kept fixed, and the
  // indices pointing to the left and right ref bracketing cameras are
  // identical.

  // The info below will eventually come from a file
  int num_cam_types = 3;
  int ref_cam_type = 0;  // Below we assume the starting cam is the ref cam

  // Image and depth topics
  std::vector<std::string> cam_names    = {"nav_cam", "haz_cam", "sci_cam"};
  std::vector<std::string> image_topics = {"/mgt/img_sampler/nav_cam/image_record",
                                           "/hw/depth_haz/extended/amplitude_int",
                                           "/hw/cam_sci/compressed"};
  std::vector<std::string> depth_topics = {"", "/hw/depth_haz/points", ""};

  // The timestamp offsets from ref cam to given cam
  std::vector<double> ref_to_cam_timestamp_offsets =
    {0.0,
     navcam_to_hazcam_timestamp_offset,
     navcam_to_hazcam_timestamp_offset - scicam_to_hazcam_timestamp_offset};

  std::cout << "--test here again!" << std::endl;
  for (size_t it = 0; it < ref_to_cam_timestamp_offsets.size(); it++) {
    std::cout << "--ref to cam offset for " << cam_names[it] << ' '
              << ref_to_cam_timestamp_offsets[it] << std::endl;
  }

  std::vector<camera::CameraParameters> cam_params = {nav_cam_params,
                                                      haz_cam_params,
                                                      sci_cam_params};

  // The transform from ref to given cam
  std::vector<Eigen::Affine3d> ref_to_cam_trans;
  ref_to_cam_trans.push_back(Eigen::Affine3d::Identity());
  ref_to_cam_trans.push_back(hazcam_to_navcam_aff_trans.inverse());
  ref_to_cam_trans.push_back(scicam_to_hazcam_aff_trans.inverse()
                             * hazcam_to_navcam_aff_trans.inverse());

  // Put in arrays, so we can optimize them
  std::vector<double> ref_to_cam_vec(num_cam_types * dense_map::NUM_RIGID_PARAMS);
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++)
    dense_map::rigid_transform_to_array
      (ref_to_cam_trans[cam_type],
       &ref_to_cam_vec[dense_map::NUM_RIGID_PARAMS * cam_type]);

  std::cout << "--test here!" << std::endl;
  for (size_t it = 0; it < ref_to_cam_trans.size(); it++) {
    std::cout << "--trans is\n" << ref_to_cam_trans[it].matrix() << std::endl;
  }

  // Depth to image transforms and scales
  std::vector<Eigen::Affine3d> depth_to_image_noscale;
  std::vector<double> depth_to_image_scales = {1.0, hazcam_depth_to_image_scale, 1.0};
  depth_to_image_noscale.push_back(Eigen::Affine3d::Identity());
  depth_to_image_noscale.push_back(hazcam_depth_to_image_noscale);
  depth_to_image_noscale.push_back(Eigen::Affine3d::Identity());
  // Put in arrays, so we can optimize them
  std::vector<double> depth_to_image_noscale_vec(num_cam_types * dense_map::NUM_RIGID_PARAMS);
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++)
    dense_map::rigid_transform_to_array
      (depth_to_image_noscale[cam_type],
       &depth_to_image_noscale_vec[dense_map::NUM_RIGID_PARAMS * cam_type]);

  // Put the intrinsics in arrays
  std::vector<double> focal_lengths(num_cam_types);
  std::vector<Eigen::Vector2d> optical_centers(num_cam_types);
  std::vector<Eigen::VectorXd> distortions(num_cam_types);
  for (int it = 0; it < num_cam_types; it++) {
    focal_lengths[it] = cam_params[it].GetFocalLength();  // average the two focal lengths
    optical_centers[it] = cam_params[it].GetOpticalOffset();
    distortions[it] = cam_params[it].GetDistortion();
  }

  // Build a map for quick access for all the messages we may need
  // TODO(oalexan1): Must the view be kept open for this to work?
  std::vector<std::string> topics;
  for (auto it = 0; it < image_topics.size(); it++)
    if (image_topics[it] != "") topics.push_back(image_topics[it]);
  for (auto it = 0; it < depth_topics.size(); it++)
    if (depth_topics[it] != "") topics.push_back(depth_topics[it]);
  rosbag::Bag bag;
  bag.open(FLAGS_ros_bag, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::map<std::string, std::vector<rosbag::MessageInstance>> bag_map;
  dense_map::indexMessages(view, bag_map);

  // A lot of care is needed here. This remembers how we travel in time
  // for each camera type so we have fewer messages to search.
  // But if a mistake is done below it will mess up this bookkeeping.
  std::vector<int> image_start_positions(num_cam_types, 0);
  std::vector<int> cloud_start_positions(num_cam_types, 0);

  //  Can subtract no more than this from ref_to_cam_timestamp_offsets[cam_type]
  //  before getting out of the bracket.
  std::vector<double> left_bound(num_cam_types, 1.0e+100);
  //  Can add no more than this from ref_to_cam_timestamp_offsets[cam_type]
  //  before getting out of the bracket.
  std::vector<double> right_bound(num_cam_types, 1.0e+100);

  std::cout << "Bracketing the images in time." << std::endl;

  // Populate the data for each camera image
  std::vector<dense_map::cameraImage> cams;
  for (int ref_it = 0; ref_it < num_ref_cams; ref_it++) {
    std::cout.precision(18);

    if (ref_cam_type != 0)
      LOG(FATAL) << "It is assumed that the ref cam type is 0.";

    for (int cam_type = ref_cam_type; cam_type < num_cam_types; cam_type++) {
      dense_map::cameraImage cam;
      bool success = false;
      if (cam_type == ref_cam_type) {
        cam.camera_type       = cam_type;
        cam.timestamp         = ref_timestamps[ref_it];
        cam.ref_timestamp     = cam.timestamp;  // the time offset is 0 between ref and itself
        cam.beg_ref_index = ref_it;
        cam.end_ref_index = ref_it;

        // Search the whole set of timestamps, so set start_pos =
        // 0. This is slower but more robust than keeping track of how
        // we move in the increasing order of time.
        int start_pos = 0;
        bool save_grayscale = true;  // for matching we will need grayscale
        double found_time = -1.0;
        // this has to succeed since we picked the ref images in the map
        if (!dense_map::lookupImage(cam.timestamp, bag_map[image_topics[cam_type]], save_grayscale,
                                    // outputs
                                    cam.image, image_start_positions[cam_type],  // care here
                                    found_time))
          LOG(FATAL) << std::fixed << std::setprecision(17)
                     << "Cannot look up camera at time " << cam.timestamp << ".\n";

        // The exact time is expected
        if (found_time != cam.timestamp)
          LOG(FATAL) << std::fixed << std::setprecision(17)
                     << "Cannot look up camera at time " << cam.timestamp << ".\n";
        std::cout.precision(18);
        std::cout << "--add ref " << found_time << std::endl;

        success = true;

      } else {
        if (ref_it + 1 >= num_ref_cams) break;  // Arrived at the end, cannot do a bracket

        // Convert the bracketing timestamps to current cam's time
        double left_timestamp = ref_timestamps[ref_it] + ref_to_cam_timestamp_offsets[cam_type];
        double right_timestamp = ref_timestamps[ref_it + 1] + ref_to_cam_timestamp_offsets[cam_type];

        if (right_timestamp <= left_timestamp)
          LOG(FATAL) << "Ref timestamps must be in increasing order.\n";

        if (right_timestamp - left_timestamp > FLAGS_bracket_len)
          continue;  // Must respect the bracket length

        std::cout.precision(18);
        std::cout << "---not ref " << left_timestamp << ' ' << right_timestamp << std::endl;

        // Find the image timestamp closest to the midpoint of the brackets. This will give
        // more room to vary the timestamp later.
        double mid_timestamp = (left_timestamp + right_timestamp)/2.0;

        // Search forward in time from image_start_positions[cam_type].
        // We will update that too later. One has to be very careful
        // with it so it does not go too far forward in time
        // so that at the next iteration we are passed what we
        // search for.
        int start_pos = image_start_positions[cam_type];  // care here
        bool save_grayscale = true;                       // for matching we will need grayscale
        double curr_timestamp = left_timestamp;           // start here
        cv::Mat best_image;
        double best_dist = 1.0e+100;
        double best_time = -1.0, found_time = -1.0;
        while (1) {
          if (found_time >= right_timestamp) break;  // out of range

          cv::Mat image;
          if (!dense_map::lookupImage(curr_timestamp, bag_map[image_topics[cam_type]], save_grayscale,
                                      // outputs
                                      image,
                                      start_pos,  // care here
                                      found_time))
            break;  // Need not succeed, but then there's no need to go on are we are at the end

          std::cout.precision(18);

          double curr_dist = std::abs(found_time - mid_timestamp);
          if (curr_dist < best_dist) {
            best_dist = curr_dist;
            best_time = found_time;
            // Update the start position for the future only if this is a good
            // solution. Otherwise we may have moved too far.
            image_start_positions[cam_type] = start_pos;
            image.copyTo(best_image);
          }

          // Go forward in time. We count on the fact that lookupImage() looks forward from given guess.
          curr_timestamp = std::nextafter(found_time, 1.01 * found_time);
        }

        if (best_time < 0.0) continue;  // bracketing failed

        // Note how we allow best_time == left_timestamp if there's no other choice
        if (best_time < left_timestamp || best_time >= right_timestamp) continue;  // no luck

        left_bound[cam_type] = std::min(left_bound[cam_type], best_time - left_timestamp);
        right_bound[cam_type] = std::min(right_bound[cam_type], right_timestamp - best_time);

        cam.camera_type       = cam_type;
        cam.timestamp         = best_time;
        cam.ref_timestamp     = best_time - ref_to_cam_timestamp_offsets[cam_type];
        cam.beg_ref_index = ref_it;
        cam.end_ref_index = ref_it + 1;
        cam.image             = best_image;

        if (cam_type == 1) {
          // Must compare raw big timestamps before and after!
          // Must compare residuals!

          double nav_start = ref_timestamps[ref_it];
          double haz_time = cam.ref_timestamp;
          double nav_end = ref_timestamps[ref_it + 1];
          // std::cout << "--xxxhaz after " << haz_time - nav_start << ' ' << nav_end - haz_time <<
          // ' ' << nav_end - nav_start << std::endl;
        }
        if (cam_type == 2) {
          double nav_start = ref_timestamps[ref_it];
          double sci_time = cam.ref_timestamp;
          double nav_end = ref_timestamps[ref_it + 1];
          // std::cout << "--xxxsci after " << sci_time - nav_start << ' ' << nav_end - sci_time <<
          // ' ' << nav_end - nav_start << std::endl;
        }

        success = true;
      }

      if (!success) continue;

      if (depth_topics[cam_type] != "") {
        double found_time = -1.0;
        cv::Mat cloud;
        // Look up the closest cloud in time (either before or after cam.timestamp)
        if (!dense_map::lookupCloud(cam.timestamp, bag_map[depth_topics[cam_type]],
                                    FLAGS_max_haz_cam_image_to_depth_timestamp_diff,
                                    // Outputs
                                    cam.depth_cloud,
                                    cloud_start_positions[cam_type],  // care here
                                    found_time)) {
          std::cout << "--fail finding cloud " << std::endl;
        } else {
          std::cout << "--success finding cloud with diff "
                    << std::abs(cam.timestamp - found_time) << std::endl;
        }
      }

      std::cout << "--success with cam of type " << cam.camera_type << std::endl;
      cams.push_back(cam);
    }  // end loop over camera types
  }    // end loop over ref images

  for (int cam_type = ref_cam_type; cam_type < num_cam_types; cam_type++) {
    if (cam_type == ref_cam_type) continue;  // bounds don't make sense here

    std::cout << "Bounds for camera " << cam_type << ": " << left_bound[cam_type]
              << ' ' << right_bound[cam_type] << std::endl;
  }

  std::cout << "--deal with adjustment!" << std::endl;
  for (size_t it = 0; it < cams.size(); it++) {
    if (cams[it].camera_type == 2) {
      dense_map::adjustImageSize(cam_params[2], cams[it].image);
    }
  }

  std::sort(cams.begin(), cams.end(), dense_map::timestampLess);

  if (FLAGS_verbose) {
  int count = 10000;
  std::vector<std::string> image_files;
  for (size_t it = 0; it < cams.size(); it++) {
    std::ostringstream oss;
    oss << count << "_" << cams[it].camera_type << ".jpg";
    std::string name = oss.str();
    std::cout << "--writing " << name << std::endl;
    cv::imwrite(name, cams[it].image);
    count++;
    image_files.push_back(name);
  }
  }

  std::cout << "--start detect" << std::endl;

  // Detect features using multiple threads
  std::vector<cv::Mat> cid_to_descriptor_map;
  std::vector<Eigen::Matrix2Xd> cid_to_keypoint_map;
  cid_to_descriptor_map.resize(cams.size());
  cid_to_keypoint_map.resize(cams.size());
  ff_common::ThreadPool thread_pool1;
  for (size_t it = 0; it < cams.size(); it++) {
    // dense_map::detectFeatures(cams[it].image, FLAGS_verbose,
    //                          &cid_to_descriptor_map[it], &cid_to_keypoint_map[it]);
    // std::cout << "--detect " << it << '/' << cams.size() << std::endl;
    thread_pool1.AddTask(&dense_map::detectFeatures, cams[it].image, FLAGS_verbose,
                         &cid_to_descriptor_map[it], &cid_to_keypoint_map[it]);
  }
  thread_pool1.Join();

  dense_map::MATCH_MAP matches;

  std::cout << "--expose the num overlap!" << std::endl;
  size_t num_overlap = 1;
  std::cout << "--overlap is " << num_overlap << std::endl;
  std::cout << "--must be >= 1" << std::endl;

  std::cout << "--temporary!!!" << std::endl;
  std::vector<std::pair<int, int> > image_pairs;
  for (size_t it1 = 0; it1 < cams.size(); it1++) {
    for (size_t it2 = it1 + 1; it2 < std::min(cams.size(), it1 + num_overlap + 1); it2++) {
      // std::cout << "--temporary!" << std::endl;
      bool is_good = (cams[it1].camera_type == 1 && cams[it2].camera_type == 0 ||
                      cams[it1].camera_type == 0 && cams[it2].camera_type == 1);
      if (!is_good) {
        continue;
      }

      std::cout << "--matching " << it1 << ' ' << it2 << std::endl;
      std::cout << "type is " << cams[it1].camera_type << ' ' << cams[it2].camera_type << std::endl;
      image_pairs.push_back(std::make_pair(it1, it2));
    }
  }

  // Find the matches using multiple threads
  ff_common::ThreadPool thread_pool2;
  std::mutex match_mutex;

  for (size_t pair_it = 0; pair_it < image_pairs.size(); pair_it++) {
    auto pair = image_pairs[pair_it];
    int left_image_it = pair.first, right_image_it = pair.second;
    // thread_pool2.AddTask(&dense_map::matchFeatures, &match_mutex,
    // std::cout << "--matching 2 " << left_image_it << ' ' << right_image_it << std::endl;
    dense_map::matchFeatures(&match_mutex,
                             left_image_it, right_image_it,
                             cid_to_descriptor_map[left_image_it],
                             cid_to_descriptor_map[right_image_it],
                             cid_to_keypoint_map[left_image_it],
                             cid_to_keypoint_map[right_image_it],
                             FLAGS_verbose, &matches[pair]);
  }
  // thread_pool2.Join();

  // If feature A in image I matches feather B in image J, which matches feature C in image K,
  // then (A, B, C) belong together into a track. Build such a track.

  std::vector<std::map<std::pair<float, float>, int>> keypoint_map(cams.size());

  int num_total_matches = 0;  // temporary

  // Give all interest points in a given image a unique id
  for (auto it = matches.begin(); it != matches.end(); it++) {
    std::pair<int, int> const& index_pair = it->first;     // alias

    int left_index = index_pair.first;
    int right_index = index_pair.second;

    dense_map::MATCH_PAIR const& match_pair = it->second;  // alias
    std::vector<dense_map::InterestPoint> const& left_ip_vec = match_pair.first;
    std::vector<dense_map::InterestPoint> const& right_ip_vec = match_pair.second;
    for (size_t ip_it = 0; ip_it < left_ip_vec.size(); ip_it++) {
      auto dist_left_ip  = std::make_pair(left_ip_vec[ip_it].x,  left_ip_vec[ip_it].y);
      auto dist_right_ip = std::make_pair(right_ip_vec[ip_it].x, right_ip_vec[ip_it].y);
      keypoint_map[left_index][dist_left_ip] = 0;
      keypoint_map[right_index][dist_right_ip] = 0;
      num_total_matches++;
    }
  }

  std::cout << "--bbb num total matches " << num_total_matches << std::endl;
  std::cout << "--why so many more matches than pid?" << std::endl;
  std::cout << "--test adding missing pairs!" << std::endl;
  std::cout << "--must do two passes!" << std::endl;

  // Give all interest points in a given image a unique id
  // And put them in a vector with the id corresponding to the interest point
  std::vector<std::vector<std::pair<float, float>>> keypoint_vec(cams.size());
  for (size_t cam_it = 0; cam_it < cams.size(); cam_it++) {
    int count = 0;
    for (auto ip_it = keypoint_map[cam_it].begin(); ip_it != keypoint_map[cam_it].end(); ip_it++) {
      ip_it->second = count;
      count++;
      std::cout << "--value " << (ip_it->first).first << ' ' << (ip_it->first).second << ' '
                << ip_it->second << std::endl;
      keypoint_vec[cam_it].push_back(ip_it->first);
      std::cout << "--size is " << keypoint_vec[cam_it].size() << std::endl;
    }
  }

  std::cout << "--write my own function! It should just remove conflicts!" << std::endl;

  openMVG::matching::PairWiseMatches match_map;
  for (auto it = matches.begin(); it != matches.end(); it++) {
    std::pair<int, int> const& index_pair = it->first;     // alias

    int left_index = index_pair.first;
    int right_index = index_pair.second;

    dense_map::MATCH_PAIR const& match_pair = it->second;  // alias
    std::vector<dense_map::InterestPoint> const& left_ip_vec = match_pair.first;
    std::vector<dense_map::InterestPoint> const& right_ip_vec = match_pair.second;

    std::vector<openMVG::matching::IndMatch> mvg_matches;

    for (size_t ip_it = 0; ip_it < left_ip_vec.size(); ip_it++) {
      auto dist_left_ip  = std::make_pair(left_ip_vec[ip_it].x,  left_ip_vec[ip_it].y);
      auto dist_right_ip = std::make_pair(right_ip_vec[ip_it].x, right_ip_vec[ip_it].y);

      // std::cout << "zzz1 " << left_index << ' ' << dist_left_ip.first << ' ' <<
      // dist_left_ip.second << ' '  << right_index << ' ' << dist_right_ip.first << ' ' <<
      // dist_right_ip.second << std::endl;

      int left_id = keypoint_map[left_index][dist_left_ip];
      int right_id = keypoint_map[right_index][dist_right_ip];
      mvg_matches.push_back(openMVG::matching::IndMatch(left_id, right_id));
    }
    match_map[index_pair] = mvg_matches;
  }

  // Build tracks
  openMVG::tracks::TracksBuilder trackBuilder;
  trackBuilder.Build(match_map);  // Build:  Efficient fusion of correspondences
  trackBuilder.Filter();          // Filter: Remove tracks that have conflict
  // trackBuilder.ExportToStream(std::cout);
  openMVG::tracks::STLMAPTracks map_tracks;
  // Export tracks as a map (each entry is a sequence of imageId and featureIndex):
  //  {TrackIndex => {(imageIndex, featureIndex), ... ,(imageIndex, featureIndex)}
  trackBuilder.ExportToSTL(map_tracks);

  // TODO(oalexan1): Print how many pairwise matches were there before
  // and after filtering tracks.

  if (map_tracks.empty())
    LOG(FATAL) << "No tracks left after filtering. Perhaps images are too dis-similar?\n";

  size_t num_elems = map_tracks.size();
  // Populate back the filtered tracks.
  std::vector<std::map<int, int> > pid_to_cid_fid;
  pid_to_cid_fid.clear();
  pid_to_cid_fid.resize(num_elems);
  size_t curr_id = 0;
  for (auto itr = map_tracks.begin(); itr != map_tracks.end(); itr++) {
    for (auto itr2 = (itr->second).begin(); itr2 != (itr->second).end(); itr2++) {
      pid_to_cid_fid[curr_id][itr2->first] = itr2->second;
    }
    curr_id++;
  }

  // The transform from every camera to the world
  std::vector<Eigen::Affine3d> world_to_cam(cams.size());
  for (size_t it = 0; it < cams.size(); it++) {
    int beg_index = cams[it].beg_ref_index;
    int end_index = cams[it].end_ref_index;
    int cam_type = cams[it].camera_type;
    // std::cout << "--ref indices " << beg_index << ' ' << end_index << std::endl;
    // std::cout << "--cam type " << cam_type << std::endl;
    world_to_cam[it] = dense_map::calc_world_to_cam_trans
      (&world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_index],
       &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * end_index],
       &ref_to_cam_vec[dense_map::NUM_RIGID_PARAMS * cam_type],
       ref_timestamps[beg_index], ref_timestamps[end_index],
       ref_to_cam_timestamp_offsets[cam_type],
       cams[it].timestamp);

    // std::cout << "--trans for camera: " << it << ' ' << world_to_cam[it].matrix() << std::endl;
  }

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    std::cout << std::endl;
    // std::cout << "pid is " << pid << std::endl;
    // std::cout << "---aaa pid size is " << pid_to_cid_fid[pid].size() << std::endl;
    // std::cout << "zzz2 ";
    for (auto cid_fid = pid_to_cid_fid[pid].begin(); cid_fid != pid_to_cid_fid[pid].end(); cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;
      // std::cout  << cid << ' ' << keypoint_vec[cid][fid].first << ' ';
      // std::cout << keypoint_vec[cid][fid].second << " ";
    }
    // std::cout << std::endl;
  }

  // Do multiview triangulation
  std::vector<Eigen::Vector3d> xyz_vec(pid_to_cid_fid.size());

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    Eigen::Vector3d xyz(0, 0, 0);
    int num = 0;

    for (auto cid_fid1 = pid_to_cid_fid[pid].begin();
         cid_fid1 != pid_to_cid_fid[pid].end(); cid_fid1++) {
      int cid1 = cid_fid1->first;
      int fid1 = cid_fid1->second;

      Eigen::Vector2d dist_ip1(keypoint_vec[cid1][fid1].first, keypoint_vec[cid1][fid1].second);
      Eigen::Vector2d undist_ip1;
      cam_params[cams[cid1].camera_type].Convert<camera::DISTORTED, camera::UNDISTORTED_C>
        (dist_ip1, &undist_ip1);

      // std::cout << "--dist undist1 " << dist_ip1.transpose() << ' ' << undist_ip1.transpose()
      //          << std::endl;

      for (auto cid_fid2 = pid_to_cid_fid[pid].begin();
           cid_fid2 != pid_to_cid_fid[pid].end(); cid_fid2++) {
        int cid2 = cid_fid2->first;
        int fid2 = cid_fid2->second;
        if (cid2 <= cid1) continue;

        Eigen::Vector2d dist_ip2(keypoint_vec[cid2][fid2].first, keypoint_vec[cid2][fid2].second);
        Eigen::Vector2d undist_ip2;
        cam_params[cams[cid2].camera_type].Convert<camera::DISTORTED, camera::UNDISTORTED_C>
          (dist_ip2, &undist_ip2);

        // std::cout << "--dist undist2 " << dist_ip2.transpose() << ' ' << undist_ip2.transpose()
        //          << std::endl;
        // std::cout << "cids are " << cid1 << ' ' << cid2 << std::endl;
        // std::cout << "--" << std::endl;
        // std::cout << "---cam1\n" << world_to_cam[cid1].matrix() << std::endl;
        // std::cout << "---cam2\n" << world_to_cam[cid2].matrix() << std::endl;

        Eigen::Vector3d xyz0
          = dense_map::TriangulatePair(cam_params[cams[cid1].camera_type].GetFocalLength(),
                                       cam_params[cams[cid2].camera_type].GetFocalLength(),
                                       world_to_cam[cid1], world_to_cam[cid2],
                                       undist_ip1, undist_ip2);

        // std::cout << "---X0 " << xyz0.transpose() << std::endl;

        xyz += xyz0;
        // std::cout << "------X " << xyz.transpose() << std::endl;
        num++;
      }
    }

    // std::cout << "---num is " << num << std::endl;
    if (num > 0) xyz /= num;

    // std::cout << "---xyz now " << xyz.transpose() << std::endl;

    xyz_vec[pid] = xyz;
    std::cout << "--xyz1 " << pid << ' ' << xyz_vec[pid].transpose() << std::endl;
    // std::cout << "---done here!" << std::endl;
  }

  std::cout << "--must do two passes!" << std::endl;
  std::cout << "--must filter by min triangulation angle and points behind camera" << std::endl;

  // std::vector<std::map<int, int>> cid_fid_to_pid;
  // sparse_mapping::InitializeCidFidToPid(cams.size(), pid_to_cid_fid, &cid_fid_to_pid);

  if (FLAGS_verbose) {
  for (auto it = matches.begin(); it != matches.end(); it++) {
    std::pair<int, int> index_pair = it->first;
    dense_map::MATCH_PAIR const& match_pair = it->second;

    int left_index = index_pair.first;
    int right_index = index_pair.second;

    std::cout << "--indices " << left_index << ' ' << right_index << std::endl;

    std::ostringstream oss1;
    oss1 << (10000 + left_index) << "_" << cams[left_index].camera_type;

    std::string left_stem = oss1.str();
    std::string left_image = left_stem + ".jpg";

    std::ostringstream oss2;
    oss2 << (10000 + right_index) << "_" << cams[right_index].camera_type;

    std::string right_stem = oss2.str();
    std::string right_image = right_stem + ".jpg";

    std::string match_file = left_stem + "__" + right_stem + ".match";

    std::cout << "Writing: " << left_image << ' ' << right_image << ' ' << match_file << std::endl;
    dense_map::writeMatchFile(match_file, match_pair.first, match_pair.second);
  }
  }

  std::cout << "--start selecting!" << std::endl;

  // Set up the variable blocks to optimize for BracketedCamError
  std::vector<int> bracketed_cam_block_sizes;
  int num_focal_lengths = 1;
  int num_distortion_params = 1;  // will be overwritten
  bracketed_cam_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_cam_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_cam_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_cam_block_sizes.push_back(dense_map::NUM_XYZ_PARAMS);
  bracketed_cam_block_sizes.push_back(dense_map::NUM_SCALAR_PARAMS);
  bracketed_cam_block_sizes.push_back(num_focal_lengths);
  bracketed_cam_block_sizes.push_back(dense_map::NUM_OPT_CTR_PARAMS);
  std::cout << "--make bracketed block sizes individual!" << std::endl;
  bracketed_cam_block_sizes.push_back(num_distortion_params);  // must be last, will be modified later

  // Set up the variable blocks to optimize for RefCamError
  std::vector<int> ref_cam_block_sizes;
  ref_cam_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  ref_cam_block_sizes.push_back(dense_map::NUM_XYZ_PARAMS);
  ref_cam_block_sizes.push_back(num_focal_lengths);
  ref_cam_block_sizes.push_back(dense_map::NUM_OPT_CTR_PARAMS);
  std::cout << "--make ref block sizes individual!" << std::endl;
  ref_cam_block_sizes.push_back(num_distortion_params);  // must be last, will be modified later

  // Set up the variable blocks to optimize for BracketedCamError
  std::vector<int> bracketed_depth_block_sizes;
  bracketed_depth_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_depth_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_depth_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_depth_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_depth_block_sizes.push_back(dense_map::NUM_SCALAR_PARAMS);
  bracketed_depth_block_sizes.push_back(dense_map::NUM_XYZ_PARAMS);
  bracketed_depth_block_sizes.push_back(dense_map::NUM_SCALAR_PARAMS);

//   // Form the problem
//   int num_xyz = 0;
//   for (auto it = matches.begin(); it != matches.end(); it++) {
//     std::pair<int, int> const& index_pair = it->first;     // alias
//     dense_map::MATCH_PAIR const& match_pair = it->second;  // alias

//     std::vector<dense_map::InterestPoint> const& left_ip_vec = match_pair.first;
//     std::vector<dense_map::InterestPoint> const& right_ip_vec = match_pair.second;
//     for (size_t ip_it = 0; ip_it < left_ip_vec.size(); ip_it++) {
//       num_xyz++;
//     }
//   }

//   std::cout << "--num xyz is " << num_xyz << std::endl;

//   xyz_vec.resize(num_xyz);

  std::cout << "must test with the ref cam having depth!" << std::endl;

  // Form the problem
  ceres::Problem problem;
  std::vector<std::string> residual_names;
  std::vector<double> residual_scales;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (auto cid_fid = pid_to_cid_fid[pid].begin();
         cid_fid != pid_to_cid_fid[pid].end(); cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;

      int cam_type = cams[cid].camera_type;
      int beg_ref_index = cams[cid].beg_ref_index;
      int end_ref_index = cams[cid].end_ref_index;

      Eigen::Vector2d dist_ip(keypoint_vec[cid][fid].first, keypoint_vec[cid][fid].second);

      if (cam_type == ref_cam_type) {
        // The cost function of projecting in the ref cam.
        ceres::CostFunction* ref_cost_function =
          dense_map::RefCamError::Create(dist_ip, ref_cam_block_sizes,
                                         cam_params[cam_type]);
        ceres::LossFunction* ref_loss_function
          = dense_map::GetLossFunction("cauchy", FLAGS_robust_threshold);

        // std::cout << "--add block" << std::endl;
        residual_names.push_back(cam_names[cam_type] + "_pix_x");
        residual_names.push_back(cam_names[cam_type] + "_pix_y");
        residual_scales.push_back(1.0);
        residual_scales.push_back(1.0);
        problem.AddResidualBlock
          (ref_cost_function, ref_loss_function,
           &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_ref_index],
           &xyz_vec[pid][0],
           &focal_lengths[cam_type],
           &optical_centers[cam_type][0],
           &distortions[cam_type][0]);

      } else {
        // Other cameras, which need bracketing
        ceres::CostFunction* bracketed_cost_function =
          dense_map::BracketedCamError::Create(dist_ip,
                                               ref_timestamps[beg_ref_index],
                                               ref_timestamps[end_ref_index],
                                               cams[cid].timestamp,
                                               bracketed_cam_block_sizes,
                                               cam_params[cam_type]);
        ceres::LossFunction* bracketed_loss_function
          = dense_map::GetLossFunction("cauchy", FLAGS_robust_threshold);

        // std::cout << "--add block" << std::endl;
        residual_names.push_back(cam_names[cam_type] + "_pix_x");
        residual_names.push_back(cam_names[cam_type] + "_pix_y");
        residual_scales.push_back(1.0);
        residual_scales.push_back(1.0);
        problem.AddResidualBlock(bracketed_cost_function, bracketed_loss_function,
                                 &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_ref_index],
                                 &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * end_ref_index],
                                 &ref_to_cam_vec[dense_map::NUM_RIGID_PARAMS * cam_type],
                                 &xyz_vec[pid][0],
                                 &ref_to_cam_timestamp_offsets[cam_type],
                                 &focal_lengths[cam_type],
                                 &optical_centers[cam_type][0],
                                 &distortions[cam_type][0]);


        Eigen::Vector3d depth_xyz(0, 0, 0);
        if (!dense_map::depthValue(cams[cid].depth_cloud, dist_ip, depth_xyz))
          continue;  // could not look up the depth value

        std::cout << "--depth xyz is " << depth_xyz.transpose() << std::endl;

        // Ensure that the depth points agree with triangulated points
        ceres::CostFunction* bracketed_depth_cost_function
          = dense_map::BracketedDepthError::Create(FLAGS_depth_weight,
                                                   depth_xyz,
                                                   ref_timestamps[beg_ref_index],
                                                   ref_timestamps[end_ref_index],
                                                   cams[cid].timestamp,
                                                   bracketed_depth_block_sizes);

        ceres::LossFunction* bracketed_depth_loss_function
          = dense_map::GetLossFunction("cauchy", FLAGS_robust_threshold);

        residual_names.push_back(cam_names[cam_type] + "_depth_x_m");
        residual_names.push_back(cam_names[cam_type] + "_depth_y_m");
        residual_names.push_back(cam_names[cam_type] + "_depth_z_m");
        residual_scales.push_back(FLAGS_depth_weight);
        residual_scales.push_back(FLAGS_depth_weight);
        residual_scales.push_back(FLAGS_depth_weight);
        problem.AddResidualBlock(bracketed_depth_cost_function,
                                 bracketed_depth_loss_function,
                                 &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_ref_index],
                                 &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * end_ref_index],
                                 &ref_to_cam_vec[dense_map::NUM_RIGID_PARAMS * cam_type],
                                 &depth_to_image_noscale_vec[dense_map::NUM_RIGID_PARAMS * cam_type],
                                 &depth_to_image_scales[cam_type],
                                 &xyz_vec[pid][0],
                                 &ref_to_cam_timestamp_offsets[cam_type]);
      }
    }
  }

  // Evaluate the residual before optimization
  double total_cost = 0.0;
  std::vector<double> residuals;
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.num_threads = 1;
  eval_options.apply_loss_function = false;  // want raw residuals
  problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);
  if (residuals.size() != residual_names.size())
    LOG(FATAL) << "There must be as many residual names as residual values.";
  if (residuals.size() != residual_scales.size())
    LOG(FATAL) << "There must be as many residual values as residual scales.";
  for (size_t it = 0; it < residuals.size(); it++)  // compensate for the scale
    residuals[it] /= residual_scales[it];

  dense_map::calc_median_residuals(residuals, residual_names, "before opt");

  if (FLAGS_verbose) {
      for (size_t it = 0; it < residuals.size(); it++)
        std::cout << "initial res " << residual_names[it] << " " << residuals[it] << std::endl;
  }

  return 0;
} // NOLINT

