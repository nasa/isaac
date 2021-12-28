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

DEFINE_string(ros_bag, "", "A ROS bag with recorded nav_cam, haz_cam, and "
              "full-resolution sci_cam data.");

DEFINE_string(sparse_map, "",
              "A registered SURF sparse map made with some of the ROS bag data, "
              "and including nav cam images closely bracketing the sci cam images.");

DEFINE_string(output_map, "", "Output file containing the updated map.");

DEFINE_string(nav_cam_topic, "/hw/cam_nav", "The nav cam topic in the bag file.");

DEFINE_string(haz_cam_points_topic, "/hw/depth_haz/points",
              "The depth point cloud topic in the bag file.");

DEFINE_string(haz_cam_intensity_topic, "/hw/depth_haz/extended/amplitude_int",
              "The depth camera intensity topic in the bag file.");

DEFINE_string(sci_cam_topic, "/hw/cam_sci/compressed", "The sci cam topic in the bag file.");

DEFINE_int32(num_overlaps, 20, "How many images (of all camera types) close and forward in "
             "time to match to given image.");

DEFINE_double(max_haz_cam_image_to_depth_timestamp_diff, 0.2,
              "Use depth haz cam clouds that are within this distance in "
              "time from the nearest haz cam intensity image.");

DEFINE_double(robust_threshold, 3.0,
              "Pixel errors much larger than this will be exponentially attenuated "
              "to affect less the cost function.");

DEFINE_int32(num_iterations, 20, "How many solver iterations to perform in calibration.");

DEFINE_double(parameter_tolerance, 1e-12, "Stop when the optimization variables change by "
              "less than this.");

DEFINE_double(bracket_len, 2.0,
              "Lookup sci and haz cam images only between consecutive nav cam images "
              "whose distance in time is no more than this (in seconds), after adjusting "
              "for the timestamp offset between these cameras. It is assumed the robot "
              "moves slowly and uniformly during this time.");

DEFINE_int32(num_opt_threads, 16, "How many threads to use in the optimization.");

DEFINE_string(sci_cam_timestamps, "",
              "Use only these sci cam timestamps. Must be "
              "a file with one timestamp per line.");

DEFINE_bool(float_sparse_map, false, "Optimize the sparse map, hence only the camera params.");

DEFINE_bool(float_scale, false,
            "If to optimize the scale of the clouds (use it if the "
            "sparse map is kept fixed), or else rescaling and registration is needed.");

DEFINE_bool(float_timestamp_offsets, false,
            "If to optimize the timestamp offsets among the cameras.");

DEFINE_double(timestamp_offsets_max_change, 1.0,
              "If floating the timestamp offsets, do not let them change by more than this "
              "(measured in seconds). Existing image bracketing acts as an additional constraint.");

DEFINE_string(nav_cam_intrinsics_to_float, "",
              "Refine 0 or more of the following intrinsics for nav_cam: focal_length, "
              "optical_center, distortion. Specify as a quoted list. "
              "For example: 'focal_length optical_center'.");

DEFINE_string(haz_cam_intrinsics_to_float, "",
              "Refine 0 or more of the following intrinsics for haz_cam: focal_length, "
              "optical_center, distortion. Specify as a quoted list. "
              "For example: 'focal_length optical_center'.");

DEFINE_string(sci_cam_intrinsics_to_float, "",
              "Refine 0 or more of the following intrinsics for sci_cam: focal_length, "
              "optical_center, distortion. Specify as a quoted list. "
              "For example: 'focal_length optical_center'.");

DEFINE_double(nav_cam_to_sci_cam_offset_override_value,
              std::numeric_limits<double>::quiet_NaN(),
              "Override the value of nav_cam_to_sci_cam_timestamp_offset from the robot config "
              "file with this value.");

DEFINE_double(depth_weight, 10.0,
              "The weight to give to depth measurements. Use a bigger number as "
              "depth errors are usually small fractions of a meter.");

DEFINE_string(mesh, "",
              "Refine the sci cam so that the sci cam texture agrees with the nav cam "
              "texture when projected on this mesh.");

DEFINE_double(mesh_weight, 25.0,
              "A larger value will give more weight to the mesh constraint. "
              "Use a bigger number as depth errors are usually small fractions of a meter.");

DEFINE_double(depth_mesh_weight, 25.0,
              "A larger value will give more weight to the constraint that the depth clouds "
              "stay close to the mesh.");

DEFINE_bool(affine_depth_to_image, false, "Assume that the depth_to_image_transform "
            "for each depth + image camera is an arbitrary affine transform rather than a "
            "rotation times a scale.");

DEFINE_int32(refiner_num_passes, 2, "How many passes of optimization to do. Outliers will be "
             "removed after every pass. Each pass will start with the previously optimized "
             "solution as an initial guess. Mesh intersections (if applicable) and ray "
             "triangulation will be recomputed before each pass.");

DEFINE_double(initial_max_reprojection_error, 100.0, "If filtering outliers, remove interest points "
              "for which the reprojection error, in pixels, is larger than this. This filtering "
              "happens when matches are created, before cameras are optimized, and a big value "
              "should be used if the initial cameras are not trusted.");

DEFINE_double(max_reprojection_error, 15.0, "If filtering outliers, remove interest points for "
              "which the reprojection error, in pixels, is larger than this. This filtering happens "
              "after the optimization pass finishes unless disabled.");

DEFINE_double(refiner_min_angle, 0.5, "If filtering outliers, remove triangulated points "
              "for which all rays converging to it make an angle (in degrees) less than this."
              "Note that some cameras in the rig may be very close to each other relative to "
              "the points the rig images, so care is needed here.");

DEFINE_bool(refiner_skip_filtering, false, "Do not do any outlier filtering.");

DEFINE_double(min_ray_dist, 0.0, "The minimum search distance from a starting point along a ray "
              "when intersecting the ray with a mesh, in meters (if applicable).");

DEFINE_double(max_ray_dist, 100.0, "The maximum search distance from a starting point along a ray "
              "when intersecting the ray with a mesh, in meters (if applicable).");

DEFINE_string(out_texture_dir, "", "If non-empty and if an input mesh was provided, "
              "project the camera images using the optimized poses onto the mesh "
              "and write the obtained .obj files in the given directory.");

DEFINE_bool(verbose, false,
            "Print the residuals and save the images and match files."
            "Stereo Pipeline's viewer can be used for visualizing these.");

namespace dense_map {

  int NUM_AFFINE_PARAMS = 12;

  // Extract a affine transform to an array of length NUM_AFFINE_PARAMS
  void affine_transform_to_array(Eigen::Affine3d const& aff, double* arr) {
    Eigen::MatrixXd M = aff.matrix();
    int count = 0;
    // The 4th row always has 0, 0, 0, 1
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 4; col++) {
        arr[count] = M(row, col);
        count++;
      }
    }
  }

  // Convert an array of length NUM_AFFINE_PARAMS to a affine
  // transform. Normalize the quaternion to make it into a rotation.
  void array_to_affine_transform(Eigen::Affine3d& aff, const double* arr) {
    Eigen::MatrixXd M = Eigen::Matrix<double, 4, 4>::Identity();

    int count = 0;
    // The 4th row always has 0, 0, 0, 1
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 4; col++) {
        M(row, col) = arr[count];
        count++;
      }
    }

    aff.matrix() = M;
  }

  // Project and save a mesh as an obj file to out_prefix.obj,
  // out_prefix.mtl, out_prefix.png.
  // TODO(oalexan1): Move to utils and call in orthoproject.cc.
  void mesh_project(mve::TriangleMesh::Ptr const& mesh,
                    std::shared_ptr<BVHTree> const& bvh_tree,
                    cv::Mat const& image,
                    Eigen::Affine3d const& world_to_cam,
                    camera::CameraParameters const& cam_params,
                    std::string const& out_prefix) {
    boost::filesystem::path out_dir = boost::filesystem::path(out_prefix).parent_path();
    std::cout << "--Parent path is " << out_dir.string() << std::endl;

    if (out_dir.string() != "") {
      if (!boost::filesystem::exists(out_dir))
        if (!boost::filesystem::create_directories(out_dir) ||
            !boost::filesystem::is_directory(out_dir))
          LOG(FATAL) << "Failed to create directory: " << out_dir;
    }

    std::cout << "--Must read all this code!" << std::endl;

    std::vector<Eigen::Vector3i> face_vec;
    std::map<int, Eigen::Vector2d> uv_map;
    int num_exclude_boundary_pixels = 0;

    std::vector<unsigned int> const& faces = mesh->get_faces();
    int num_faces = faces.size();
    std::vector<double> smallest_cost_per_face(num_faces, 1.0e+100);

    camera::CameraModel cam(world_to_cam, cam_params);

    // Find the UV coordinates and the faces having them
    dense_map::projectTexture(mesh, bvh_tree, image, cam, num_exclude_boundary_pixels,
                              smallest_cost_per_face, face_vec, uv_map);

    std::string obj_str;
    dense_map::formObjCustomUV(mesh, face_vec, uv_map, out_prefix, obj_str);

    std::string mtl_str;
    dense_map::formMtl(out_prefix, mtl_str);

    std::string obj_file = out_prefix + ".obj";
    std::cout << "Writing: " << obj_file << std::endl;
    std::ofstream obj_handle(obj_file);
    obj_handle << obj_str;
    obj_handle.close();

    std::string mtl_file = out_prefix + ".mtl";
    std::cout << "Writing: " << mtl_file << std::endl;
    std::ofstream mtl_handle(mtl_file);
    mtl_handle << mtl_str;
    mtl_handle.close();

    std::string texture_file = out_prefix + ".png";
    std::cout << "Writing: " << texture_file << std::endl;
    cv::imwrite(texture_file, image);
  }

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
    // std::cout.precision(18);
    // std::cout << "ref to cam off " << ref_to_cam_offset << std::endl;
    // std::cout << "--ref to cam trans\n" << ref_to_cam_aff.matrix() << std::endl;

    // Covert from cam time to ref time and normalize.  It is very
    // important that below we subtract the big numbers from each
    // other first, which are the timestamps, then subtract whatever
    // else is necessary. Otherwise we get problems with numerical
    // precision with CERES.
    double alpha = ((cam_stamp - beg_ref_stamp) - ref_to_cam_offset)
      / (end_ref_stamp - beg_ref_stamp);

    if (beg_ref_stamp == end_ref_stamp)
      alpha = 0.0;  // handle division by zero

    // std::cout << "--alpha " << alpha << std::endl;
    if (alpha < 0.0 || alpha > 1.0) LOG(FATAL) << "Out of bounds in interpolation.\n";

    // Interpolate at desired time
    Eigen::Affine3d interp_world_to_ref_aff
      = dense_map::linearInterp(alpha, beg_world_to_ref_aff, end_world_to_ref_aff);

    Eigen::Affine3d interp_world_to_cam_aff = ref_to_cam_aff * interp_world_to_ref_aff;

    // std::cout << "final trans\n" << interp_world_to_cam_aff.matrix() << std::endl;

    return interp_world_to_cam_aff;
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
    if (m_block_sizes.size() != 5                   ||
        m_block_sizes[0]     != NUM_RIGID_PARAMS    ||
        m_block_sizes[1]     != NUM_XYZ_PARAMS      ||
        m_block_sizes[2]     != m_num_focal_lengths ||
        m_block_sizes[3]     != NUM_OPT_CTR_PARAMS  ||
        m_block_sizes[4]     != 1  // This will be overwritten shortly
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
        (m_block_sizes[3] != NUM_RIGID_PARAMS  && m_block_sizes[3] != NUM_AFFINE_PARAMS) ||
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
    if (m_block_sizes[3] == NUM_AFFINE_PARAMS)
      array_to_affine_transform(depth_to_image, parameters[3]);
    else
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

// An error function minimizing the product of a given weight and the
// error between a mesh point and a transformed measured depth point. The
// depth point needs to be transformed to world coordinates first. For
// that one has to do pose interpolation.
struct BracketedDepthMeshError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BracketedDepthMeshError(double weight,
                          Eigen::Vector3d const& meas_depth_xyz,
                          Eigen::Vector3d const& mesh_xyz,
                          double left_ref_stamp, double right_ref_stamp, double cam_stamp,
                          std::vector<int> const& block_sizes):
    m_weight(weight),
    m_meas_depth_xyz(meas_depth_xyz),
    m_mesh_xyz(mesh_xyz),
    m_left_ref_stamp(left_ref_stamp),
    m_right_ref_stamp(right_ref_stamp),
    m_cam_stamp(cam_stamp),
    m_block_sizes(block_sizes) {
    // Sanity check
    if (m_block_sizes.size() != 6 ||
        m_block_sizes[0] != NUM_RIGID_PARAMS  ||
        m_block_sizes[1] != NUM_RIGID_PARAMS  ||
        m_block_sizes[2] != NUM_RIGID_PARAMS  ||
        (m_block_sizes[3] != NUM_RIGID_PARAMS  && m_block_sizes[3] != NUM_AFFINE_PARAMS) ||
        m_block_sizes[4] != NUM_SCALAR_PARAMS ||
        m_block_sizes[5] != NUM_SCALAR_PARAMS) {
      LOG(FATAL) << "BracketedDepthMeshError: The block sizes were not set up properly.\n";
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
                              parameters[5][0],  // ref_to_cam_offset
                              m_cam_stamp);

    // The current transform from the depth point cloud to the camera image
    Eigen::Affine3d depth_to_image;
    if (m_block_sizes[3] == NUM_AFFINE_PARAMS)
      array_to_affine_transform(depth_to_image, parameters[3]);
    else
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

    // Compute the residuals
    for (size_t it = 0; it < NUM_XYZ_PARAMS; it++) {
      residuals[it] = m_weight * (m_mesh_xyz[it] - M[it]);
      // std::cout << "--mesh residual " << residuals[it] << std::endl;
    }

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(double weight,
                                     Eigen::Vector3d const& meas_depth_xyz,
                                     Eigen::Vector3d const& mesh_xyz,
                                     double left_ref_stamp, double right_ref_stamp,
                                     double cam_stamp, std::vector<int> const& block_sizes) {
    ceres::DynamicNumericDiffCostFunction<BracketedDepthMeshError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<BracketedDepthMeshError>
      (new BracketedDepthMeshError(weight, meas_depth_xyz, mesh_xyz,
                                   left_ref_stamp, right_ref_stamp,
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
  Eigen::Vector3d m_mesh_xyz;                  // Point on preexisting mesh
  double m_left_ref_stamp, m_right_ref_stamp;  // left and right ref cam timestamps
  double m_cam_stamp;                          // Current cam timestamp
  std::vector<int> m_block_sizes;
};  // End class BracketedDepthMeshError

// An error function minimizing the product of a given weight and the
// error between a triangulated point and a measured depth point for
// the ref camera. The depth point needs to be transformed to world
// coordinates first.
struct RefDepthError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RefDepthError(double weight, Eigen::Vector3d const& meas_depth_xyz,
                std::vector<int> const& block_sizes):
    m_weight(weight),
    m_meas_depth_xyz(meas_depth_xyz),
    m_block_sizes(block_sizes) {
    // Sanity check
    if (m_block_sizes.size() != 4 ||
        m_block_sizes[0] != NUM_RIGID_PARAMS ||
        (m_block_sizes[1] != NUM_RIGID_PARAMS  && m_block_sizes[1] != NUM_AFFINE_PARAMS) ||
        m_block_sizes[2] != NUM_SCALAR_PARAMS ||
        m_block_sizes[3] != NUM_XYZ_PARAMS) {
      LOG(FATAL) << "RefDepthError: The block sizes were not set up properly.\n";
    }
  }

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  bool operator()(double const* const* parameters, double* residuals) const {
    // Current world to camera transform
    Eigen::Affine3d world_to_cam;
    array_to_rigid_transform(world_to_cam, parameters[0]);

    // The current transform from the depth point cloud to the camera image
    Eigen::Affine3d depth_to_image;
    if (m_block_sizes[1] == NUM_AFFINE_PARAMS)
      array_to_affine_transform(depth_to_image, parameters[1]);
    else
      array_to_rigid_transform(depth_to_image, parameters[1]);

    // Apply the scale
    double depth_to_image_scale = parameters[2][0];
    depth_to_image.linear() *= depth_to_image_scale;
    // std::cout << "--depth to image:\n" << depth_to_image.matrix() << std::endl;

    // std::cout << "--meas pt " << m_meas_depth_xyz.transpose() << std::endl;

    // Convert from depth cloud coordinates to cam coordinates
    Eigen::Vector3d M = depth_to_image * m_meas_depth_xyz;

    // std::cout << "--image meas pt " << M.transpose() << std::endl;

    // Convert to world coordinates
    M = world_to_cam.inverse() * M;
    // std::cout << "--depth in world coords " << M.transpose() << std::endl;

    // Triangulated world point
    Eigen::Vector3d X(parameters[3][0], parameters[3][1], parameters[3][2]);
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
                                     std::vector<int> const& block_sizes) {
    ceres::DynamicNumericDiffCostFunction<RefDepthError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<RefDepthError>
      (new RefDepthError(weight, meas_depth_xyz, block_sizes));

    cost_function->SetNumResiduals(NUM_XYZ_PARAMS);

    for (size_t i = 0; i < block_sizes.size(); i++)
      cost_function->AddParameterBlock(block_sizes[i]);

    return cost_function;
  }

 private:
  double m_weight;                             // How much weight to give to this constraint
  Eigen::Vector3d m_meas_depth_xyz;            // Measured depth measurement
  std::vector<int> m_block_sizes;
};  // End class RefDepthError

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
    for (int it = 0; it < NUM_XYZ_PARAMS; it++)
      residuals[it] = m_weight * (parameters[0][it] - m_ref_xyz[it]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Eigen::Vector3d const& ref_xyz,
                                     std::vector<int> const& block_sizes,
                                     double weight) {
    ceres::DynamicNumericDiffCostFunction<XYZError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<XYZError>
      (new XYZError(ref_xyz, block_sizes, weight));

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

  std::cout << "The 25, 50, 75, and 100th percentile residual stats " << tag << std::endl;
  for (auto it = stats.begin(); it != stats.end(); it++) {
    std::string const& name = it->first;
    std::vector<double> vals = stats[name];  // make a copy
    std::sort(vals.begin(), vals.end());

    int len = vals.size();

    int it1 = static_cast<int>(0.25 * len);
    int it2 = static_cast<int>(0.50 * len);
    int it3 = static_cast<int>(0.75 * len);
    int it4 = static_cast<int>(len - 1);

    if (len == 0)
      std::cout << name << ": " << "none";
    else
      std::cout << std::setprecision(8)
                << name << ": " << vals[it1] << ' ' << vals[it2] << ' '
                << vals[it3] << ' ' << vals[it4];
    std::cout << " (" << len << " residuals)" << std::endl;
  }
}

  // TODO(oalexan1): This needs to be handled better.
  void adjustImageSize(camera::CameraParameters const& cam_params, cv::Mat & image) {
    int raw_image_cols = image.cols;
    int raw_image_rows = image.rows;
    int calib_image_cols = cam_params.GetDistortedSize()[0];
    int calib_image_rows = cam_params.GetDistortedSize()[1];

    int factor = raw_image_cols / calib_image_cols;

    if ((raw_image_cols != calib_image_cols * factor) ||
        (raw_image_rows != calib_image_rows * factor)) {
      LOG(FATAL) << "Image width and height are: " << raw_image_cols << ' ' << raw_image_rows
                 << "\n"
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

  typedef std::map<std::pair<int, int>, dense_map::MATCH_PAIR> MATCH_MAP;

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

    // The timestamp of the closest cloud for this image
    double cloud_timestamp;

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

  // Form textured meshes for given images
  void mesh_project_images(std::vector<std::string> const& cam_names,
                           std::vector<camera::CameraParameters> const& cam_params,
                           std::vector<dense_map::cameraImage> const& cam_images,
                           std::vector<Eigen::Affine3d> const& world_to_cam,
                           mve::TriangleMesh::Ptr const& mesh,
                           std::shared_ptr<BVHTree> const& bvh_tree,
                           std::string const& out_dir) {
    if (cam_names.size() != cam_params.size())
      LOG(FATAL) << "There must be as many camera names as sets of camera parameters.\n";
    if (cam_images.size() != world_to_cam.size())
      LOG(FATAL) << "There must be as many camera images as camera poses.\n";
    if (out_dir.empty())
      LOG(FATAL) << "The output directory is empty.\n";
  }

  // Rebuild the map.
  // TODO(oalexan1): This must be integrated in astrobee.
  void RebuildMap(std::string const& map_file,  // Will be used for temporary saving of aux data
                  camera::CameraParameters const& cam_params,
                  boost::shared_ptr<sparse_mapping::SparseMap> sparse_map) {
    std::string rebuild_detector = "SURF";
    std::cout << "Rebuilding map with " << rebuild_detector << " detector.";

    // Copy some data to make sure it does not get lost on resetting things below
    std::vector<Eigen::Affine3d>    world_to_ref_t = sparse_map->cid_to_cam_t_global_;
    std::vector<std::map<int, int>> pid_to_cid_fid = sparse_map->pid_to_cid_fid_;

    // Ensure the new camera parameters are set
    sparse_map->SetCameraParameters(cam_params);

    std::cout << "Detecting features.";
    sparse_map->DetectFeatures();

    std::cout << "Matching features.";
    // Borrow from the original map which images should be matched with which.
    sparse_map->cid_to_cid_.clear();
    for (size_t p = 0; p < pid_to_cid_fid.size(); p++) {
      std::map<int, int> const& track = pid_to_cid_fid[p];  // alias
      for (std::map<int, int>::const_iterator it1 = track.begin();
           it1 != track.end() ; it1++) {
        for (std::map<int, int>::const_iterator it2 = it1;
             it2 != track.end() ; it2++) {
          if (it1->first != it2->first) {
            // Never match an image with itself
            sparse_map->cid_to_cid_[it1->first].insert(it2->first);
          }
        }
      }
    }

    sparse_mapping::MatchFeatures(sparse_mapping::EssentialFile(map_file),
                                  sparse_mapping::MatchesFile(map_file), sparse_map.get());
    for (size_t i = 0; i < world_to_ref_t.size(); i++)
      sparse_map->SetFrameGlobalTransform(i, world_to_ref_t[i]);

    // Wipe file that is no longer needed
    try {
      std::remove(sparse_mapping::EssentialFile(map_file).c_str());
    }catch(...) {}

    std::cout << "Building tracks.";
    bool rm_invalid_xyz = true;  // by now cameras are good, so filter out bad stuff
    sparse_mapping::BuildTracks(rm_invalid_xyz,
                                sparse_mapping::MatchesFile(map_file),
                                sparse_map.get());

    // It is essential that during re-building we do not vary the
    // cameras. Those were usually computed with a lot of SURF features,
    // while rebuilding is usually done with many fewer ORGBRISK
    // features.
    bool fix_cameras = true;
    if (fix_cameras)
      std::cout << "Performing bundle adjustment with fixed cameras.";
    else
      std::cout << "Performing bundle adjustment while floating cameras.";

    sparse_mapping::BundleAdjust(fix_cameras, sparse_map.get());
  }

  // TODO(oalexan1): Move this to utils.
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

  // Compute the transforms from the world to every camera, using pose interpolation
  // if necessary.
  void calc_world_to_cam_transforms(  // Inputs
    std::vector<dense_map::cameraImage> const& cams, std::vector<double> const& world_to_ref_vec,
    std::vector<double> const& ref_timestamps, std::vector<double> const& ref_to_cam_vec,
    std::vector<double> const& ref_to_cam_timestamp_offsets,
    // Output
    std::vector<Eigen::Affine3d>& world_to_cam) {
    if (ref_to_cam_vec.size() / dense_map::NUM_RIGID_PARAMS != ref_to_cam_timestamp_offsets.size())
      LOG(FATAL) << "Must have as many transforms to reference as timestamp offsets.\n";
    if (world_to_ref_vec.size() / dense_map::NUM_RIGID_PARAMS != ref_timestamps.size())
      LOG(FATAL) << "Must have as many reference timestamps as reference cameras.\n";

    world_to_cam.resize(cams.size());

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
    }
    return;
  }

  // Match features while assuming that the input cameras can be used to filter out
  // outliers by reprojection error.
  void matchFeaturesWithCams(std::mutex* match_mutex, int left_image_index, int right_image_index,
                             camera::CameraParameters const& left_params,
                             camera::CameraParameters const& right_params,
                             Eigen::Affine3d const& left_world_to_cam,
                             Eigen::Affine3d const& right_world_to_cam,
                             double reprojection_error,
                             cv::Mat const& left_descriptors, cv::Mat const& right_descriptors,
                             Eigen::Matrix2Xd const& left_keypoints,
                             Eigen::Matrix2Xd const& right_keypoints,
                             bool verbose,
                             // output
                             MATCH_PAIR* matches) {
    // Match by using descriptors first
    std::vector<cv::DMatch> cv_matches;
    interest_point::FindMatches(left_descriptors, right_descriptors, &cv_matches);

    // Do filtering
    std::vector<cv::Point2f> left_vec;
    std::vector<cv::Point2f> right_vec;
    std::vector<cv::DMatch> filtered_cv_matches;
    for (size_t j = 0; j < cv_matches.size(); j++) {
      int left_ip_index = cv_matches.at(j).queryIdx;
      int right_ip_index = cv_matches.at(j).trainIdx;

      Eigen::Vector2d dist_left_ip(left_keypoints.col(left_ip_index)[0],
                                   left_keypoints.col(left_ip_index)[1]);

      Eigen::Vector2d dist_right_ip(right_keypoints.col(right_ip_index)[0],
                                    right_keypoints.col(right_ip_index)[1]);

      Eigen::Vector2d undist_left_ip;
      Eigen::Vector2d undist_right_ip;
      left_params.Convert<camera::DISTORTED,  camera::UNDISTORTED_C>
        (dist_left_ip, &undist_left_ip);
      right_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>
        (dist_right_ip, &undist_right_ip);

      Eigen::Vector3d X =
        dense_map::TriangulatePair(left_params.GetFocalLength(), right_params.GetFocalLength(),
                                   left_world_to_cam, right_world_to_cam,
                                   undist_left_ip, undist_right_ip);

      // Project back into the cameras
      Eigen::Vector3d left_cam_X = left_world_to_cam * X;
      Eigen::Vector2d undist_left_pix
        = left_params.GetFocalVector().cwiseProduct(left_cam_X.hnormalized());
      Eigen::Vector2d dist_left_pix;
      left_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undist_left_pix,
                                                                    &dist_left_pix);

      Eigen::Vector3d right_cam_X = right_world_to_cam * X;
      Eigen::Vector2d undist_right_pix
        = right_params.GetFocalVector().cwiseProduct(right_cam_X.hnormalized());
      Eigen::Vector2d dist_right_pix;
      right_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undist_right_pix,
                                                                    &dist_right_pix);

      // Filter out points whose reprojection error is too big
      bool is_good = ((dist_left_ip - dist_left_pix).norm() <= reprojection_error &&
                      (dist_right_ip - dist_right_pix).norm() <= reprojection_error);

      // If any values above are Inf or NaN, is_good will be false as well
      if (!is_good) continue;

      // Get the keypoints from the good matches
      left_vec.push_back(cv::Point2f(left_keypoints.col(left_ip_index)[0],
                                     left_keypoints.col(left_ip_index)[1]));
      right_vec.push_back(cv::Point2f(right_keypoints.col(right_ip_index)[0],
                                      right_keypoints.col(right_ip_index)[1]));

      filtered_cv_matches.push_back(cv_matches[j]);
    }

    if (left_vec.empty()) return;

    // Filter using geometry constraints
    // These may need some tweaking but works reasonably well.
    double ransacReprojThreshold = 20.0;
    cv::Mat inlier_mask;
    int maxIters = 10000;
    double confidence = 0.8;

    // affine2D works better than homography
    // cv::Mat H = cv::findHomography(left_vec, right_vec, cv::RANSAC,
    // ransacReprojThreshold, inlier_mask, maxIters, confidence);
    cv::Mat H = cv::estimateAffine2D(left_vec, right_vec, inlier_mask, cv::RANSAC,
                                     ransacReprojThreshold, maxIters, confidence);

    std::vector<InterestPoint> left_ip, right_ip;
    for (size_t j = 0; j < filtered_cv_matches.size(); j++) {
      int left_ip_index = filtered_cv_matches.at(j).queryIdx;
      int right_ip_index = filtered_cv_matches.at(j).trainIdx;

      if (inlier_mask.at<uchar>(j, 0) == 0) continue;

      cv::Mat left_desc = left_descriptors.row(left_ip_index);
      cv::Mat right_desc = right_descriptors.row(right_ip_index);

      InterestPoint left;
      left.setFromCvKeypoint(left_keypoints.col(left_ip_index), left_desc);

      InterestPoint right;
      right.setFromCvKeypoint(right_keypoints.col(right_ip_index), right_desc);

      left_ip.push_back(left);
      right_ip.push_back(right);
    }

    // Update the shared variable using a lock
    match_mutex->lock();

    // Print the verbose message inside the lock, otherwise the text
    // may get messed up.
    if (verbose)
      std::cout << "Number of matches for pair "
                << left_image_index << ' ' << right_image_index << ": "
                << left_ip.size() << std::endl;

    *matches = std::make_pair(left_ip, right_ip);
    match_mutex->unlock();
  }

  // Find if a given feature is an inlier, and take care to check that
  // the bookkeeping is correct.
  int getMapValue(std::vector<std::map<int, std::map<int, int>>> const& pid_cid_fid,
                size_t pid, int cid, int fid) {
    if (pid_cid_fid.size() <= pid)
      LOG(FATAL) << "Current pid is out of range.\n";

    auto& cid_fid_map = pid_cid_fid[pid];  // alias
    auto cid_it = cid_fid_map.find(cid);
    if (cid_it == cid_fid_map.end()) LOG(FATAL) << "Current cid it out of range.\n";

    auto& fid_map = cid_it->second;  // alias
    auto fid_it = fid_map.find(fid);
    if (fid_it == fid_map.end()) LOG(FATAL) << "Current fid is out of range.\n";

    return fid_it->second;
  }

  // Set a feature to be an outlier, and take care to check that
  // the bookkeeping is correct.
  void setMapValue(std::vector<std::map<int, std::map<int, int>>> & pid_cid_fid,
                size_t pid, int cid, int fid, int val) {
    if (pid_cid_fid.size() <= pid)
      LOG(FATAL) << "Current pid is out of range.\n";

    auto& cid_fid_map = pid_cid_fid[pid];  // alias
    auto cid_it = cid_fid_map.find(cid);
    if (cid_it == cid_fid_map.end()) LOG(FATAL) << "Current cid it out of range.\n";

    auto& fid_map = cid_it->second;  // alias
    auto fid_it = fid_map.find(fid);
    if (fid_it == fid_map.end()) LOG(FATAL) << "Current fid is out of range.\n";

    fid_it->second = val;
  }

}  // namespace dense_map

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  std::cout.precision(17);  // to be able to print timestamps

  if (FLAGS_ros_bag.empty())
    LOG(FATAL) << "The bag file was not specified.";
  if (FLAGS_sparse_map.empty())
    LOG(FATAL) << "The input sparse map was not specified.";

  if (FLAGS_output_map.empty())
    LOG(FATAL) << "The output sparse map was not specified.";

  if (FLAGS_robust_threshold <= 0.0)
    LOG(FATAL) << "The robust threshold must be positive.\n";

  if (FLAGS_bracket_len <= 0.0) LOG(FATAL) << "Bracket length must be positive.";

  if (FLAGS_num_overlaps < 1) LOG(FATAL) << "Number of overlaps must be positive.";

  if (FLAGS_timestamp_offsets_max_change < 0)
    LOG(FATAL) << "The timestamp offsets must be non-negative.";

  if (FLAGS_refiner_min_angle <= 0.0)
    LOG(FATAL) << "The min triangulation angle must be positive.\n";

  if (FLAGS_mesh_weight <= 0.0)
    LOG(FATAL) << "The mesh weight must be positive.\n";

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

  std::vector<camera::CameraParameters> cam_params;
  std::vector<Eigen::Affine3d>          ref_to_cam_trans;
  std::vector<double>                   ref_to_cam_timestamp_offsets;
  Eigen::Affine3d                       nav_cam_to_body_trans;
  Eigen::Affine3d                       haz_cam_depth_to_image_transform;
  dense_map::readConfigFile(  // Inputs
    cam_names, "nav_cam_transform", "haz_cam_depth_to_image_transform",
    // Outputs
    cam_params, ref_to_cam_trans, ref_to_cam_timestamp_offsets, nav_cam_to_body_trans,
    haz_cam_depth_to_image_transform);
  std::vector<camera::CameraParameters> orig_cam_params = cam_params;

  if (!std::isnan(FLAGS_nav_cam_to_sci_cam_offset_override_value)) {
    for (size_t it = 0; it < cam_names.size(); it++) {
      if (cam_names[it] == "sci_cam") {
        double new_val = FLAGS_nav_cam_to_sci_cam_offset_override_value;
        std::cout << "Overriding the value " << ref_to_cam_timestamp_offsets[it]
                  << " of nav_cam_to_sci_cam_timestamp_offset with: " << new_val << std::endl;
        ref_to_cam_timestamp_offsets[it] = new_val;
      }
    }
  }

  mve::TriangleMesh::Ptr mesh;
  std::shared_ptr<mve::MeshInfo> mesh_info;
  std::shared_ptr<tex::Graph> graph;
  std::shared_ptr<BVHTree> bvh_tree;
  if (FLAGS_mesh != "") dense_map::loadMeshBuildTree(FLAGS_mesh, mesh, mesh_info, graph, bvh_tree);

  // If desired to process only specific timestamps
  std::set<double> sci_cam_timestamps_to_use;
  if (FLAGS_sci_cam_timestamps != "") {
    std::ifstream ifs(FLAGS_sci_cam_timestamps.c_str());
    double val;
    while (ifs >> val) sci_cam_timestamps_to_use.insert(val);
  }

  double haz_cam_depth_to_image_scale
    = pow(haz_cam_depth_to_image_transform.matrix().determinant(), 1.0 / 3.0);

  // Since we will keep the scale fixed, vary the part of the transform without
  // the scale, while adding the scale each time before the transform is applied
  Eigen::Affine3d haz_cam_depth_to_image_noscale = haz_cam_depth_to_image_transform;
  haz_cam_depth_to_image_noscale.linear() /= haz_cam_depth_to_image_scale;

  // Read the sparse map
  boost::shared_ptr<sparse_mapping::SparseMap> sparse_map =
    boost::shared_ptr<sparse_mapping::SparseMap>(new sparse_mapping::SparseMap(FLAGS_sparse_map));

  // TODO(oalexan1): All this timestamp reading logic below should be in a function

  // Parse the ref timestamps from the sparse map
  // We assume the sparse map image names are the timestamps.
  std::vector<double> ref_timestamps;
  const std::vector<std::string>& sparse_map_images = sparse_map->cid_to_filename_;
  ref_timestamps.resize(sparse_map_images.size());
  for (size_t cid = 0; cid < sparse_map_images.size(); cid++) {
    double timestamp = dense_map::fileNameToTimestamp(sparse_map_images[cid]);
    ref_timestamps[cid] = timestamp;
  }
  if (ref_timestamps.empty()) LOG(FATAL) << "No sparse map timestamps found.";

  // Will optimize the nav cam poses as part of the process
  std::vector<Eigen::Affine3d> & world_to_ref_t = sparse_map->cid_to_cam_t_global_;  // alias

  // Put transforms of the reference cameras in a vector. We will optimize them.
  int num_ref_cams = world_to_ref_t.size();
  if (world_to_ref_t.size() != ref_timestamps.size())
    LOG(FATAL) << "Must have as many ref cam timestamps as ref cameras.\n";
  std::vector<double> world_to_ref_vec(num_ref_cams * dense_map::NUM_RIGID_PARAMS);
  for (int cid = 0; cid < num_ref_cams; cid++)
    dense_map::rigid_transform_to_array(world_to_ref_t[cid],
                                        &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * cid]);


  std::vector<std::set<double>> cam_timestamps_to_use = {std::set<double>(),
                                                         std::set<double>(),
                                                         sci_cam_timestamps_to_use};

  // Which intrinsics from which cameras to float
  std::vector<std::set<std::string>> intrinsics_to_float(num_cam_types);
  dense_map::parse_intrinsics_to_float(FLAGS_nav_cam_intrinsics_to_float, intrinsics_to_float[0]);
  dense_map::parse_intrinsics_to_float(FLAGS_haz_cam_intrinsics_to_float, intrinsics_to_float[1]);
  dense_map::parse_intrinsics_to_float(FLAGS_sci_cam_intrinsics_to_float, intrinsics_to_float[2]);

  // Put in arrays, so we can optimize them
  std::vector<double> ref_to_cam_vec(num_cam_types * dense_map::NUM_RIGID_PARAMS);
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++)
    dense_map::rigid_transform_to_array
      (ref_to_cam_trans[cam_type],
       &ref_to_cam_vec[dense_map::NUM_RIGID_PARAMS * cam_type]);

  // Set up the variable blocks to optimize for BracketedDepthError
  int num_depth_params = dense_map::NUM_RIGID_PARAMS;
  if (FLAGS_affine_depth_to_image) num_depth_params = dense_map::NUM_AFFINE_PARAMS;

  // Depth to image transforms and scales
  std::vector<Eigen::Affine3d> depth_to_image_noscale;
  std::vector<double> depth_to_image_scales = {1.0, haz_cam_depth_to_image_scale, 1.0};
  depth_to_image_noscale.push_back(Eigen::Affine3d::Identity());
  depth_to_image_noscale.push_back(haz_cam_depth_to_image_noscale);
  depth_to_image_noscale.push_back(Eigen::Affine3d::Identity());

  // Put in arrays, so we can optimize them
  std::vector<double> depth_to_image_noscale_vec(num_cam_types * num_depth_params);
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
    if (FLAGS_affine_depth_to_image)
      dense_map::affine_transform_to_array(depth_to_image_noscale[cam_type],
                                           &depth_to_image_noscale_vec[num_depth_params * cam_type]);
    else
      dense_map::rigid_transform_to_array(depth_to_image_noscale[cam_type],
                                          &depth_to_image_noscale_vec[num_depth_params * cam_type]);
  }

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

  //  Cannot add a (positive) value more this to
  //  ref_to_cam_timestamp_offsets[cam_type] before getting out of the
  //  bracket.
  std::vector<double> upper_bound(num_cam_types, 1.0e+100);

  //  Cannot add a (negative) value less than this to
  //  ref_to_cam_timestamp_offsets[cam_type] before getting out of the
  //  bracket.
  std::vector<double> lower_bound(num_cam_types, -1.0e+100);

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

        // See if to skip this timestamp
        if (!cam_timestamps_to_use[cam_type].empty() &&
            cam_timestamps_to_use[cam_type].find(cam.timestamp) ==
            cam_timestamps_to_use[cam_type].end())
          continue;

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
          if (!dense_map::lookupImage(curr_timestamp, bag_map[image_topics[cam_type]],
                                      save_grayscale,
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

          // Go forward in time. We count on the fact that
          // lookupImage() looks forward from given guess.
          curr_timestamp = std::nextafter(found_time, 1.01 * found_time);
        }

        if (best_time < 0.0) continue;  // bracketing failed

        // Note how we allow best_time == left_timestamp if there's no other choice
        if (best_time < left_timestamp || best_time >= right_timestamp) continue;  // no luck

        cam.camera_type   = cam_type;
        cam.timestamp     = best_time;
        cam.ref_timestamp = best_time - ref_to_cam_timestamp_offsets[cam_type];
        cam.beg_ref_index = ref_it;
        cam.end_ref_index = ref_it + 1;
        cam.image         = best_image;

        upper_bound[cam_type] = std::min(upper_bound[cam_type], best_time - left_timestamp);
        lower_bound[cam_type] = std::max(lower_bound[cam_type], best_time - right_timestamp);

        // TODO(oalexan1): Wipe this temporary block
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

        // See if to skip this timestamp
        if (!cam_timestamps_to_use[cam_type].empty() &&
            cam_timestamps_to_use[cam_type].find(cam.timestamp) ==
            cam_timestamps_to_use[cam_type].end())
          continue;

        success = true;
      }

      if (!success) continue;

      if (depth_topics[cam_type] != "") {
        cam.cloud_timestamp = -1.0;  // will change
        cv::Mat cloud;
        // Look up the closest cloud in time (either before or after cam.timestamp)
        // This need not succeed.
        dense_map::lookupCloud(cam.timestamp, bag_map[depth_topics[cam_type]],
                               FLAGS_max_haz_cam_image_to_depth_timestamp_diff,
                               // Outputs
                               cam.depth_cloud,
                               cloud_start_positions[cam_type],  // care here
                               cam.cloud_timestamp);
      }

      cams.push_back(cam);
    }  // end loop over camera types
  }    // end loop over ref images

  std::cout << "Allowed timestamp offset range while respecting the bracket for given cameras:\n";
  for (int cam_type = ref_cam_type; cam_type < num_cam_types; cam_type++) {
    if (cam_type == ref_cam_type) continue;  // bounds don't make sense here

    // So far we had the relative change. Now add the actual offset to get the max allowed offset.
    lower_bound[cam_type] += ref_to_cam_timestamp_offsets[cam_type];
    upper_bound[cam_type] += ref_to_cam_timestamp_offsets[cam_type];

    std::cout << std::setprecision(8) << cam_names[cam_type] << ": [" << lower_bound[cam_type]
              << ", " << upper_bound[cam_type] << "]\n";
  }

  if (FLAGS_float_timestamp_offsets) {
    std::cout << "Given the user constraint the timestamp offsets will float in these ranges:\n";
    for (int cam_type = ref_cam_type; cam_type < num_cam_types; cam_type++) {
      if (cam_type == ref_cam_type) continue;  // bounds don't make sense here

      lower_bound[cam_type] = std::max(lower_bound[cam_type],
                                       ref_to_cam_timestamp_offsets[cam_type]
                                       - FLAGS_timestamp_offsets_max_change);

      upper_bound[cam_type] = std::min(upper_bound[cam_type],
                                       ref_to_cam_timestamp_offsets[cam_type]
                                       + FLAGS_timestamp_offsets_max_change);

      // Tighten a bit to ensure we don't exceed things when we add
      // and subtract timestamps later.  Note that timestamps are
      // measured in seconds and fractions of a second since epoch and
      // can be quite large so precision loss can easily happen.
      lower_bound[cam_type] += 1.0e-5;
      upper_bound[cam_type] -= 1.0e-5;

      std::cout << std::setprecision(8) << cam_names[cam_type] << ": [" << lower_bound[cam_type]
                << ", " << upper_bound[cam_type] << "]\n";
    }
  }

  std::cout << "--Deal with adjustment!" << std::endl;
  for (size_t it = 0; it < cams.size(); it++) {
    if (cams[it].camera_type == 2) {
      dense_map::adjustImageSize(cam_params[2], cams[it].image);
    }
  }

  // Sort by the timestamp in reference camera time
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

  std::cout << "Detecting features." << std::endl;

  // Detect features using multiple threads
  std::vector<cv::Mat> cid_to_descriptor_map;
  std::vector<Eigen::Matrix2Xd> cid_to_keypoint_map;
  cid_to_descriptor_map.resize(cams.size());
  cid_to_keypoint_map.resize(cams.size());
  ff_common::ThreadPool thread_pool1;
  for (size_t it = 0; it < cams.size(); it++) {
    thread_pool1.AddTask(&dense_map::detectFeatures, cams[it].image, FLAGS_verbose,
                         &cid_to_descriptor_map[it], &cid_to_keypoint_map[it]);
  }
  thread_pool1.Join();

  dense_map::MATCH_MAP matches;

  std::vector<std::pair<int, int> > image_pairs;
  for (size_t it1 = 0; it1 < cams.size(); it1++) {
    for (size_t it2 = it1 + 1; it2 < std::min(cams.size(), it1 + FLAGS_num_overlaps + 1); it2++) {
      image_pairs.push_back(std::make_pair(it1, it2));
    }
  }

  {
    // The transform from the world to every camera
    std::vector<Eigen::Affine3d> world_to_cam;
    calc_world_to_cam_transforms(  // Inputs
      cams, world_to_ref_vec, ref_timestamps, ref_to_cam_vec, ref_to_cam_timestamp_offsets,
      // Output
      world_to_cam);

    std::cout << "Matching features." << std::endl;
    ff_common::ThreadPool thread_pool2;
    std::mutex match_mutex;
    for (size_t pair_it = 0; pair_it < image_pairs.size(); pair_it++) {
      auto pair = image_pairs[pair_it];
      int left_image_it = pair.first, right_image_it = pair.second;
      bool verbose = true;
      if (FLAGS_refiner_skip_filtering) {
        thread_pool2.AddTask(&dense_map::matchFeatures,
                             &match_mutex,
                             left_image_it, right_image_it,
                             cid_to_descriptor_map[left_image_it],
                             cid_to_descriptor_map[right_image_it],
                             cid_to_keypoint_map[left_image_it],
                             cid_to_keypoint_map[right_image_it],
                             verbose, &matches[pair]);
      } else {
        thread_pool2.AddTask(&dense_map::matchFeaturesWithCams,
                             &match_mutex,
                             left_image_it, right_image_it,
                             cam_params[cams[left_image_it].camera_type],
                             cam_params[cams[right_image_it].camera_type],
                             world_to_cam[left_image_it],
                             world_to_cam[right_image_it],
                             FLAGS_initial_max_reprojection_error,
                             cid_to_descriptor_map[left_image_it],
                             cid_to_descriptor_map[right_image_it],
                             cid_to_keypoint_map[left_image_it],
                             cid_to_keypoint_map[right_image_it],
                             verbose, &matches[pair]);
      }
    }
    thread_pool2.Join();
  }

  // If feature A in image I matches feather B in image J, which matches feature C in image K,
  // then (A, B, C) belong together into a track. Build such a track.

  std::vector<std::map<std::pair<float, float>, int>> keypoint_map(cams.size());

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
    }
  }

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
      // std::cout << "--value " << (ip_it->first).first << ' ' << (ip_it->first).second << ' '
      //          << ip_it->second << std::endl;
      keypoint_vec[cam_it].push_back(ip_it->first);
      // std::cout << "--size is " << keypoint_vec[cam_it].size() << std::endl;
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

  std::cout << "Find other things which need deallocation!" << std::endl;
  std::cout << "De-allocate images and point clouds when no longer needed!" << std::endl;

  // not needed anymore and take up a lot of RAM
  matches.clear(); matches = dense_map::MATCH_MAP();
  keypoint_map.clear(); keypoint_map.shrink_to_fit();
  cid_to_descriptor_map.clear(); cid_to_descriptor_map.shrink_to_fit();
  cid_to_keypoint_map.clear(); cid_to_keypoint_map.shrink_to_fit();

  std::vector<std::map<int, int> > pid_to_cid_fid;
  {
    // Build tracks
    // De-allocate these as soon as not needed to save memory
    openMVG::tracks::TracksBuilder trackBuilder;
    trackBuilder.Build(match_map);  // Build:  Efficient fusion of correspondences
    trackBuilder.Filter();          // Filter: Remove tracks that have conflict
    // trackBuilder.ExportToStream(std::cout);
    // Export tracks as a map (each entry is a sequence of imageId and featureIndex):
    //  {TrackIndex => {(imageIndex, featureIndex), ... ,(imageIndex, featureIndex)}
    openMVG::tracks::STLMAPTracks map_tracks;
    trackBuilder.ExportToSTL(map_tracks);
    match_map = openMVG::matching::PairWiseMatches();  // wipe this, no longer needed

    // TODO(oalexan1): Print how many pairwise matches were there before
    // and after filtering tracks.

    if (map_tracks.empty())
      LOG(FATAL) << "No tracks left after filtering. Perhaps images are too dis-similar?\n";

    size_t num_elems = map_tracks.size();
    // Populate the filtered tracks
    pid_to_cid_fid.clear();
    pid_to_cid_fid.resize(num_elems);
    size_t curr_id = 0;
    for (auto itr = map_tracks.begin(); itr != map_tracks.end(); itr++) {
      for (auto itr2 = (itr->second).begin(); itr2 != (itr->second).end(); itr2++) {
        pid_to_cid_fid[curr_id][itr2->first] = itr2->second;
      }
      curr_id++;
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
  bracketed_cam_block_sizes.push_back(num_distortion_params);  // will be modified later

  // Set up the variable blocks to optimize for RefCamError
  std::vector<int> ref_cam_block_sizes;
  ref_cam_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  ref_cam_block_sizes.push_back(dense_map::NUM_XYZ_PARAMS);
  ref_cam_block_sizes.push_back(num_focal_lengths);
  ref_cam_block_sizes.push_back(dense_map::NUM_OPT_CTR_PARAMS);
  std::cout << "--make ref block sizes individual!" << std::endl;
  ref_cam_block_sizes.push_back(num_distortion_params);  // will be modified later

  // Set up variable blocks to optimize for BracketedDepthError
  std::vector<int> bracketed_depth_block_sizes;
  bracketed_depth_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_depth_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_depth_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_depth_block_sizes.push_back(num_depth_params);
  bracketed_depth_block_sizes.push_back(dense_map::NUM_SCALAR_PARAMS);
  bracketed_depth_block_sizes.push_back(dense_map::NUM_XYZ_PARAMS);
  bracketed_depth_block_sizes.push_back(dense_map::NUM_SCALAR_PARAMS);

  // Set up the variable blocks to optimize for BracketedDepthMeshError
  std::vector<int> bracketed_depth_mesh_block_sizes;
  bracketed_depth_mesh_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_depth_mesh_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_depth_mesh_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  bracketed_depth_mesh_block_sizes.push_back(num_depth_params);
  bracketed_depth_mesh_block_sizes.push_back(dense_map::NUM_SCALAR_PARAMS);
  bracketed_depth_mesh_block_sizes.push_back(dense_map::NUM_SCALAR_PARAMS);

  // Set up the variable blocks to optimize for RefDepthError
  std::vector<int> ref_depth_block_sizes;
  ref_depth_block_sizes.push_back(dense_map::NUM_RIGID_PARAMS);
  ref_depth_block_sizes.push_back(num_depth_params);
  ref_depth_block_sizes.push_back(dense_map::NUM_SCALAR_PARAMS);
  ref_depth_block_sizes.push_back(dense_map::NUM_XYZ_PARAMS);

  // Set up the variable blocks to optimize for the mesh xyz
  std::vector<int> mesh_block_sizes;
  mesh_block_sizes.push_back(dense_map::NUM_XYZ_PARAMS);

  std::cout << "---Must start passes here" << std::endl;
  std::cout << "--After each pass must update all structures!" << std::endl;

  // For a given fid = pid_to_cid_fid[pid][cid], the value
  // pid_cid_fid_inlier[pid][cid][fid] will be non-zero only if this
  // pixel is an inlier. Originally all pixels are inliers. Once an
  // inlier becomes an outlier, it never becomes an inlier again.
  std::vector<std::map<int, std::map<int, int>>> pid_cid_fid_inlier;
  pid_cid_fid_inlier.resize(pid_to_cid_fid.size());
  if (!FLAGS_refiner_skip_filtering) {
    for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
      for (auto cid_fid = pid_to_cid_fid[pid].begin(); cid_fid != pid_to_cid_fid[pid].end();
           cid_fid++) {
        int cid = cid_fid->first;
        int fid = cid_fid->second;
        pid_cid_fid_inlier[pid][cid][fid] = 1;
      }
    }
  }

  for (int pass = 0; pass < FLAGS_refiner_num_passes; pass++) {
    std::cout << "\nOptimization pass " << pass + 1 << " / " << FLAGS_refiner_num_passes << std::endl;

    // The transform from the world to every camera
    std::vector<Eigen::Affine3d> world_to_cam;
    calc_world_to_cam_transforms(  // Inputs
      cams, world_to_ref_vec, ref_timestamps, ref_to_cam_vec, ref_to_cam_timestamp_offsets,
      // Output
      world_to_cam);

    // Do multiview triangulation
    std::vector<Eigen::Vector3d> xyz_vec(pid_to_cid_fid.size());

    for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
      Eigen::Vector3d mesh_xyz(0, 0, 0);
      int num = 0;

      std::vector<double> focal_length_vec;
      std::vector<Eigen::Affine3d> world_to_cam_vec;
      std::vector<Eigen::Vector2d> pix_vec;

      for (auto cid_fid = pid_to_cid_fid[pid].begin(); cid_fid != pid_to_cid_fid[pid].end();
           cid_fid++) {
        int cid = cid_fid->first;
        int fid = cid_fid->second;

        // Triangulate inliers only
        if (!FLAGS_refiner_skip_filtering &&
            !dense_map::getMapValue(pid_cid_fid_inlier, pid, cid, fid))
          continue;

        Eigen::Vector2d dist_ip(keypoint_vec[cid][fid].first, keypoint_vec[cid][fid].second);
        Eigen::Vector2d undist_ip;
        cam_params[cams[cid].camera_type].Convert<camera::DISTORTED, camera::UNDISTORTED_C>
          (dist_ip, &undist_ip);

        focal_length_vec.push_back(cam_params[cams[cid].camera_type].GetFocalLength());
        world_to_cam_vec.push_back(world_to_cam[cid]);
        pix_vec.push_back(undist_ip);
      }

      if (!FLAGS_refiner_skip_filtering && pix_vec.size() < 2) {
        // If after outlier filtering less than two rays are left, can't triangulate.
        // Must set all features for this pid to outliers.
        for (auto cid_fid = pid_to_cid_fid[pid].begin(); cid_fid != pid_to_cid_fid[pid].end();
             cid_fid++) {
          int cid = cid_fid->first;
          int fid = cid_fid->second;
          dense_map::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
        }

        // Nothing else to do
        continue;
      }

      // Triangulate n rays emanating from given undistorted and centered pixels
      xyz_vec[pid] = dense_map::Triangulate(focal_length_vec, world_to_cam_vec, pix_vec);
      // std::cout << "--xyz " << pid << ' ' << xyz_vec[pid].transpose() << std::endl;
    }

    std::cout << "must test with the ref cam having depth!" << std::endl;

    // For a given fid = pid_to_cid_fid[pid][cid],
    // the value pid_cid_fid_to_residual[pid][cid][fid] will be the index in the array
    // of residuals (look only at pixel residuals). This structure is populated only for
    // inliers, so its size changes at each pass.
    std::vector<std::map<int, std::map<int, int>>> pid_cid_fid_to_residual;
    if (!FLAGS_refiner_skip_filtering) pid_cid_fid_to_residual.resize(pid_to_cid_fid.size());

    // Form the problem
    ceres::Problem problem;
    std::vector<std::string> residual_names;
    std::vector<double> residual_scales;
    for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
      for (auto cid_fid = pid_to_cid_fid[pid].begin();
           cid_fid != pid_to_cid_fid[pid].end(); cid_fid++) {
        int cid = cid_fid->first;
        int fid = cid_fid->second;

        // Deal with inliers only
        if (!FLAGS_refiner_skip_filtering &&
            !dense_map::getMapValue(pid_cid_fid_inlier, pid, cid, fid))
          continue;

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

          // Remember the index of the residuals about to create
          if (!FLAGS_refiner_skip_filtering)
            pid_cid_fid_to_residual[pid][cid][fid] = residual_names.size();

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

          if (!FLAGS_float_sparse_map) {
            problem.SetParameterBlockConstant
              (&world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_ref_index]);
          }

          Eigen::Vector3d depth_xyz(0, 0, 0);
          if (!dense_map::depthValue(cams[cid].depth_cloud, dist_ip, depth_xyz))
            continue;  // could not look up the depth value

          // std::cout << "--depth xyz is " << depth_xyz.transpose() << std::endl;

          // The constraint that the depth points agree with triangulated points
          ceres::CostFunction* ref_depth_cost_function
            = dense_map::RefDepthError::Create(FLAGS_depth_weight,
                                               depth_xyz,
                                               ref_depth_block_sizes);

          ceres::LossFunction* ref_depth_loss_function
            = dense_map::GetLossFunction("cauchy", FLAGS_robust_threshold);

          residual_names.push_back("depth_tri_x_m");
          residual_names.push_back("depth_tri_y_m");
          residual_names.push_back("depth_tri_z_m");
          residual_scales.push_back(FLAGS_depth_weight);
          residual_scales.push_back(FLAGS_depth_weight);
          residual_scales.push_back(FLAGS_depth_weight);
          problem.AddResidualBlock
            (ref_depth_cost_function,
             ref_depth_loss_function,
             &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_ref_index],
             &depth_to_image_noscale_vec[num_depth_params * cam_type],
             &depth_to_image_scales[cam_type],
             &xyz_vec[pid][0]);

          if (!FLAGS_float_sparse_map)
            problem.SetParameterBlockConstant
              (&world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_ref_index]);
          if (!FLAGS_float_scale || FLAGS_affine_depth_to_image)
            problem.SetParameterBlockConstant(&depth_to_image_scales[cam_type]);

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

          // Remember the index of the residuals about to create
          if (!FLAGS_refiner_skip_filtering)
            pid_cid_fid_to_residual[pid][cid][fid] = residual_names.size();

          residual_names.push_back(cam_names[cam_type] + "_pix_x");
          residual_names.push_back(cam_names[cam_type] + "_pix_y");
          residual_scales.push_back(1.0);
          residual_scales.push_back(1.0);
          problem.AddResidualBlock
            (bracketed_cost_function, bracketed_loss_function,
             &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_ref_index],
             &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * end_ref_index],
             &ref_to_cam_vec[dense_map::NUM_RIGID_PARAMS * cam_type],
             &xyz_vec[pid][0],
             &ref_to_cam_timestamp_offsets[cam_type],
             &focal_lengths[cam_type],
             &optical_centers[cam_type][0],
             &distortions[cam_type][0]);

          if (!FLAGS_float_sparse_map) {
            problem.SetParameterBlockConstant
              (&world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_ref_index]);
            problem.SetParameterBlockConstant
              (&world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * end_ref_index]);
          }
          if (!FLAGS_float_timestamp_offsets) {
            problem.SetParameterBlockConstant(&ref_to_cam_timestamp_offsets[cam_type]);
          } else {
            problem.SetParameterLowerBound(&ref_to_cam_timestamp_offsets[cam_type], 0,
                                           lower_bound[cam_type]);
            problem.SetParameterUpperBound(&ref_to_cam_timestamp_offsets[cam_type], 0,
                                           upper_bound[cam_type]);
          }

          Eigen::Vector3d depth_xyz(0, 0, 0);
          if (!dense_map::depthValue(cams[cid].depth_cloud, dist_ip, depth_xyz))
            continue;  // could not look up the depth value

          // std::cout << "--depth xyz is " << depth_xyz.transpose() << std::endl;

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

          residual_names.push_back(cam_names[cam_type] + "_depth_tri_x_m");
          residual_names.push_back(cam_names[cam_type] + "_depth_tri_y_m");
          residual_names.push_back(cam_names[cam_type] + "_depth_tri_z_m");
          residual_scales.push_back(FLAGS_depth_weight);
          residual_scales.push_back(FLAGS_depth_weight);
          residual_scales.push_back(FLAGS_depth_weight);
          problem.AddResidualBlock
            (bracketed_depth_cost_function,
             bracketed_depth_loss_function,
             &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_ref_index],
             &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * end_ref_index],
             &ref_to_cam_vec[dense_map::NUM_RIGID_PARAMS * cam_type],
             &depth_to_image_noscale_vec[num_depth_params * cam_type],
             &depth_to_image_scales[cam_type],
             &xyz_vec[pid][0],
             &ref_to_cam_timestamp_offsets[cam_type]);

          // TODO(oalexan1): This code repeats too much. Need to keep a hash
          // of sparse map pointers.
          if (!FLAGS_float_sparse_map) {
            problem.SetParameterBlockConstant
              (&world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_ref_index]);
            problem.SetParameterBlockConstant
              (&world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * end_ref_index]);
          }
          if (!FLAGS_float_scale || FLAGS_affine_depth_to_image)
            problem.SetParameterBlockConstant(&depth_to_image_scales[cam_type]);
          if (!FLAGS_float_timestamp_offsets) {
            problem.SetParameterBlockConstant(&ref_to_cam_timestamp_offsets[cam_type]);
          } else {
            problem.SetParameterLowerBound(&ref_to_cam_timestamp_offsets[cam_type], 0,
                                           lower_bound[cam_type]);
            problem.SetParameterUpperBound(&ref_to_cam_timestamp_offsets[cam_type], 0,
                                           upper_bound[cam_type]);
          }
        }
      }
    }

    if (FLAGS_mesh != "") {
      // Add the mesh constraint

      for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
        Eigen::Vector3d mesh_xyz(0, 0, 0);
        int num = 0;

        for (auto cid_fid = pid_to_cid_fid[pid].begin(); cid_fid != pid_to_cid_fid[pid].end();
             cid_fid++) {
          int cid = cid_fid->first;
          int fid = cid_fid->second;

          // Deal with inliers only
          if (!FLAGS_refiner_skip_filtering &&
              !dense_map::getMapValue(pid_cid_fid_inlier, pid, cid, fid))
            continue;

          Eigen::Vector2d dist_ip(keypoint_vec[cid][fid].first, keypoint_vec[cid][fid].second);
          Eigen::Vector2d undist_ip;
          cam_params[cams[cid].camera_type].Convert<camera::DISTORTED, camera::UNDISTORTED_C>
            (dist_ip, &undist_ip);

          // Intersect the ray with the mesh
          bool have_mesh_intersection = false;
          Eigen::Vector3d mesh_intersect(0.0, 0.0, 0.0);
          have_mesh_intersection
            = dense_map::ray_mesh_intersect(undist_ip, cam_params[cams[cid].camera_type],
                                            world_to_cam[cid], mesh, bvh_tree,
                                            FLAGS_min_ray_dist, FLAGS_max_ray_dist,
                                            // Output
                                            mesh_intersect);

          if (have_mesh_intersection) {
            Eigen::Vector3d depth_xyz(0, 0, 0);
            if (dense_map::depthValue(cams[cid].depth_cloud, dist_ip, depth_xyz)) {
              int cam_type = cams[cid].camera_type;
              int beg_ref_index = cams[cid].beg_ref_index;
              int end_ref_index = cams[cid].end_ref_index;
              if (cam_type != ref_cam_type) {
                // TODO(oalexan1): Review this
                // TODO(oalexan1): What to do when cam_type == ref_cam_type?
                // Ensure that the depth points agree with mesh points
                ceres::CostFunction* bracketed_depth_mesh_cost_function
                  = dense_map::BracketedDepthMeshError::Create(FLAGS_depth_mesh_weight,
                                                               depth_xyz,
                                                               mesh_intersect,
                                                               ref_timestamps[beg_ref_index],
                                                               ref_timestamps[end_ref_index],
                                                               cams[cid].timestamp,
                                                               bracketed_depth_mesh_block_sizes);

                ceres::LossFunction* bracketed_depth_mesh_loss_function
                  = dense_map::GetLossFunction("cauchy", FLAGS_robust_threshold);

                residual_names.push_back("depth_mesh_x_m");
                residual_names.push_back("depth_mesh_y_m");
                residual_names.push_back("depth_mesh_z_m");
                residual_scales.push_back(FLAGS_depth_mesh_weight);
                residual_scales.push_back(FLAGS_depth_mesh_weight);
                residual_scales.push_back(FLAGS_depth_mesh_weight);
                problem.AddResidualBlock
                  (bracketed_depth_mesh_cost_function,
                   bracketed_depth_mesh_loss_function,
                   &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_ref_index],
                   &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * end_ref_index],
                   &ref_to_cam_vec[dense_map::NUM_RIGID_PARAMS * cam_type],
                   &depth_to_image_noscale_vec[num_depth_params * cam_type],
                   &depth_to_image_scales[cam_type],
                   &ref_to_cam_timestamp_offsets[cam_type]);

                // TODO(oalexan1): This code repeats too much. Need to keep a hash
                // of sparse map pointers.
                if (!FLAGS_float_sparse_map) {
                  problem.SetParameterBlockConstant
                    (&world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * beg_ref_index]);
                  problem.SetParameterBlockConstant
                    (&world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * end_ref_index]);
                }
                if (!FLAGS_float_scale || FLAGS_affine_depth_to_image)
                  problem.SetParameterBlockConstant(&depth_to_image_scales[cam_type]);
                if (!FLAGS_float_timestamp_offsets) {
                  problem.SetParameterBlockConstant(&ref_to_cam_timestamp_offsets[cam_type]);
                } else {
                  problem.SetParameterLowerBound(&ref_to_cam_timestamp_offsets[cam_type], 0,
                                                 lower_bound[cam_type]);
                  problem.SetParameterUpperBound(&ref_to_cam_timestamp_offsets[cam_type], 0,
                                                 upper_bound[cam_type]);
                }
              }
            }
          }

          if (have_mesh_intersection) {
            mesh_xyz += mesh_intersect;
            num += 1;
          }
        }

        if (num >= 1) {
          mesh_xyz /= num;

          // std::cout << "--num and mesh xyz is " << num << ' ' << mesh_xyz.transpose() << std::endl;

          ceres::CostFunction* mesh_cost_function =
            dense_map::XYZError::Create(mesh_xyz, mesh_block_sizes, FLAGS_mesh_weight);

          ceres::LossFunction* mesh_loss_function =
            dense_map::GetLossFunction("cauchy", FLAGS_robust_threshold);

          problem.AddResidualBlock(mesh_cost_function, mesh_loss_function,
                                   &xyz_vec[pid][0]);

          residual_names.push_back("mesh_tri_x_m");
          residual_names.push_back("mesh_tri_y_m");
          residual_names.push_back("mesh_tri_z_m");

          residual_scales.push_back(FLAGS_mesh_weight);
          residual_scales.push_back(FLAGS_mesh_weight);
          residual_scales.push_back(FLAGS_mesh_weight);
        }
      }
      // std::cout << "--xyz1 " << pid << ' ' << xyz_vec[pid].transpose() << std::endl;
    }

    // See which intrinsics from which cam to float or keep fixed
    for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
      if (intrinsics_to_float[cam_type].find("focal_length") == intrinsics_to_float[cam_type].end()) {
        // std::cout << "For " << cam_names[cam_type] << " not floating focal_length." << std::endl;
        problem.SetParameterBlockConstant(&focal_lengths[cam_type]);
      } else {
        // std::cout << "For " << cam_names[cam_type] << " floating focal_length." << std::endl;
      }
      if (intrinsics_to_float[cam_type].find("optical_center") ==
          intrinsics_to_float[cam_type].end()) {
        // std::cout << "For " << cam_names[cam_type] << " not floating optical_center." <<
        // std::endl;
        problem.SetParameterBlockConstant(&optical_centers[cam_type][0]);
      } else {
        // std::cout << "For " << cam_names[cam_type] << " floating optical_center." << std::endl;
      }
      if (intrinsics_to_float[cam_type].find("distortion") == intrinsics_to_float[cam_type].end()) {
        // std::cout << "For " << cam_names[cam_type] << " not floating distortion." << std::endl;
        problem.SetParameterBlockConstant(&distortions[cam_type][0]);
      } else {
        // std::cout << "For " << cam_names[cam_type] << " floating distortion." << std::endl;
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
        std::cout << "Initial res " << residual_names[it] << " " << residuals[it] << std::endl;
    }

    // Solve the problem
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.num_threads = FLAGS_num_opt_threads;  // The result is more predictable with one thread
    options.max_num_iterations = FLAGS_num_iterations;
    options.minimizer_progress_to_stdout = true;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.parameter_tolerance = FLAGS_parameter_tolerance;
    ceres::Solve(options, &problem, &summary);

    // Evaluate the residual after optimization
    eval_options.num_threads = 1;
    eval_options.apply_loss_function = false;  // want raw residuals
    problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);
    if (residuals.size() != residual_names.size())
      LOG(FATAL) << "There must be as many residual names as residual values.";
    if (residuals.size() != residual_scales.size())
      LOG(FATAL) << "There must be as many residual values as residual scales.";
    for (size_t it = 0; it < residuals.size(); it++)  // compensate for the scale
      residuals[it] /= residual_scales[it];
    dense_map::calc_median_residuals(residuals, residual_names, "after opt");
    if (FLAGS_verbose) {
      for (size_t it = 0; it < residuals.size(); it++)
        std::cout << "Final res " << residual_names[it] << " " << residuals[it] << std::endl;
    }

    // Flag outliers
    if (!FLAGS_refiner_skip_filtering) {
      // Before computing outliers by triangulation angle must recompute all the cameras,
      // given the optimized parameters
      calc_world_to_cam_transforms(  // Inputs
        cams, world_to_ref_vec, ref_timestamps, ref_to_cam_vec, ref_to_cam_timestamp_offsets,
        // Output
        world_to_cam);

      // Must deal with outliers by triangulation angle before
      // removing outliers by reprojection error, as the latter will
      // exclude some rays which form the given triangulated points.
      int num_outliers_by_angle = 0, num_total_features = 0;
      for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
        // Find the largest angle among any two intersecting rays
        double max_rays_angle = 0.0;

        for (auto cid_fid1 = pid_to_cid_fid[pid].begin();
             cid_fid1 != pid_to_cid_fid[pid].end(); cid_fid1++) {
          int cid1 = cid_fid1->first;
          int fid1 = cid_fid1->second;

          // Deal with inliers only
          if (!dense_map::getMapValue(pid_cid_fid_inlier, pid, cid1, fid1)) continue;

          num_total_features++;

          Eigen::Vector3d cam_ctr1 = (world_to_cam[cid1].inverse()) * Eigen::Vector3d(0, 0, 0);
          Eigen::Vector3d ray1 = xyz_vec[pid] - cam_ctr1;
          ray1.normalize();

          for (auto cid_fid2 = pid_to_cid_fid[pid].begin();
               cid_fid2 != pid_to_cid_fid[pid].end(); cid_fid2++) {
            int cid2 = cid_fid2->first;
            int fid2 = cid_fid2->second;

            // Look at each cid and next cids
            if (cid2 <= cid1)
              continue;

            // Deal with inliers only
            if (!dense_map::getMapValue(pid_cid_fid_inlier, pid, cid2, fid2)) continue;

            Eigen::Vector3d cam_ctr2 = (world_to_cam[cid2].inverse()) * Eigen::Vector3d(0, 0, 0);
            Eigen::Vector3d ray2 = xyz_vec[pid] - cam_ctr2;
            ray2.normalize();

            double curr_angle = (180.0 / M_PI) * acos(ray1.dot(ray2));

            if (std::isnan(curr_angle) || std::isinf(curr_angle)) continue;

            max_rays_angle = std::max(max_rays_angle, curr_angle);
          }
        }

        if (max_rays_angle >= FLAGS_refiner_min_angle)
          continue;  // This is a good triangulated point, with large angle of convergence

        // Flag as outliers all the features for this cid
        for (auto cid_fid = pid_to_cid_fid[pid].begin();
             cid_fid != pid_to_cid_fid[pid].end(); cid_fid++) {
          int cid = cid_fid->first;
          int fid = cid_fid->second;

          // Deal with inliers only
          if (!dense_map::getMapValue(pid_cid_fid_inlier, pid, cid, fid)) continue;

          num_outliers_by_angle++;
          dense_map::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
        }
      }
      std::cout << std::setprecision(4) << "Removed " << num_outliers_by_angle
                << " outlier features with small angle of convergence, out of "
                << num_total_features << " ("
                << (100.0 * num_outliers_by_angle) / num_total_features << " %)\n";

      int num_outliers_reproj = 0;
      num_total_features = 0;  // reusing this variable
      for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
        for (auto cid_fid = pid_to_cid_fid[pid].begin();
             cid_fid != pid_to_cid_fid[pid].end(); cid_fid++) {
          int cid = cid_fid->first;
          int fid = cid_fid->second;

          // Deal with inliers only
          if (!dense_map::getMapValue(pid_cid_fid_inlier, pid, cid, fid)) continue;

          num_total_features++;

          // Find the pixel residuals
          size_t residual_index = dense_map::getMapValue(pid_cid_fid_to_residual, pid, cid, fid);
          if (residuals.size() <= residual_index + 1) LOG(FATAL) << "Too few residuals.\n";

          double res_x = residuals[residual_index + 0];
          double res_y = residuals[residual_index + 1];
          // NaN values will never be inliers if the comparison is set as below
          bool is_good = (Eigen::Vector2d(res_x, res_y).norm() <= FLAGS_max_reprojection_error);
          if (!is_good) {
            num_outliers_reproj++;
            dense_map::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
          }
        }
      }
      std::cout << std::setprecision(4) << "Removed " << num_outliers_reproj
                << " outlier features using reprojection error, out of " << num_total_features
                << " features (" << (100.0 * num_outliers_reproj) / num_total_features << " %)\n";
    }

    // Copy back the reference transforms
    for (int cid = 0; cid < num_ref_cams; cid++)
      dense_map::array_to_rigid_transform(world_to_ref_t[cid],
                                          &world_to_ref_vec[dense_map::NUM_RIGID_PARAMS * cid]);

    // Copy back the optimized intrinsics
    for (int it = 0; it < num_cam_types; it++) {
      cam_params[it].SetFocalLength(Eigen::Vector2d(focal_lengths[it], focal_lengths[it]));
      cam_params[it].SetOpticalOffset(optical_centers[it]);
      cam_params[it].SetDistortion(distortions[it]);
    }

    // If the nav cam did not get optimized, go back to the solution
    // with two focal lengths, rather than the one with one focal length
    // solved by this solver (as the average of the two).  The two focal
    // lengths are very similar, but it is not worth modifying the
    // camera model we don't plan to optimize.
    if (FLAGS_nav_cam_intrinsics_to_float == "" || FLAGS_num_iterations == 0)
      cam_params[ref_cam_type] = orig_cam_params[ref_cam_type];

    // Copy back the optimized extrinsics
    for (int cam_type = 0; cam_type < num_cam_types; cam_type++)
      dense_map::array_to_rigid_transform
        (ref_to_cam_trans[cam_type],
         &ref_to_cam_vec[dense_map::NUM_RIGID_PARAMS * cam_type]);

    // Copy back the depth to image transforms without scales
    for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
      if (FLAGS_affine_depth_to_image)
        dense_map::array_to_affine_transform
          (depth_to_image_noscale[cam_type],
           &depth_to_image_noscale_vec[num_depth_params * cam_type]);
      else
        dense_map::array_to_rigid_transform(
          depth_to_image_noscale[cam_type],
          &depth_to_image_noscale_vec[num_depth_params * cam_type]);
    }
  }

  // Update the optimized depth to image (for haz cam only)
  int cam_type = 1;  // haz cam
  haz_cam_depth_to_image_scale = depth_to_image_scales[cam_type];
  haz_cam_depth_to_image_transform = depth_to_image_noscale[cam_type];
  haz_cam_depth_to_image_transform.linear() *= haz_cam_depth_to_image_scale;

  dense_map::updateConfigFile(cam_names, "haz_cam_depth_to_image_transform",
                              cam_params, ref_to_cam_trans,
                              ref_to_cam_timestamp_offsets,
                              haz_cam_depth_to_image_transform);

  if (FLAGS_num_iterations > 0 &&
      (FLAGS_float_sparse_map || FLAGS_nav_cam_intrinsics_to_float != "")) {
    std::cout << "Either the sparse map intrinsics or cameras got modified. "
              << "The map must be rebuilt." << std::endl;

    dense_map::RebuildMap(FLAGS_output_map,  // Will be used for temporary saving of aux data
                          cam_params[ref_cam_type], sparse_map);
  }

  std::cout << "Writing: " << FLAGS_output_map << std::endl;
  sparse_map->Save(FLAGS_output_map);

  std::cout << "--must save matches after all filtering!" << std::endl;

  return 0;
} // NOLINT

