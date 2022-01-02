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

#ifndef DENSE_MAP_UTILS_H_
#define DENSE_MAP_UTILS_H_

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <config_reader/config_reader.h>
#include <camera/camera_params.h>

#include <boost/shared_ptr.hpp>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace dense_map {

const int NUM_SCALAR_PARAMS  = 1;
const int NUM_OPT_CTR_PARAMS = 2;  // optical center in x and y
const int NUM_RESIDUALS      = 2;  // Same as pixel size
const int NUM_XYZ_PARAMS     = 3;
const int NUM_RIGID_PARAMS   = 7;  // quaternion (4) + translation (3)
const int NUM_AFFINE_PARAMS = 12;  // 3x3 matrix (9) + translation (3)

// A  function to split a string like 'optical_center focal_length' into
// its two constituents.
void parse_intrinsics_to_float(std::string const& intrinsics_to_float,
                                 std::set<std::string>& intrinsics_to_float_set);

// A  function to split a string like 'haz_cam sci_cam' into
// its two constituents and validate against the list of known cameras.
void parse_extrinsics_to_float(std::vector<std::string> const& cam_names,
                               std::string const& depth_to_image_name,
                               std::string const& extrinsics_to_float,
                               std::set<std::string>& extrinsics_to_float_set);

// Extract a rigid transform to an array of length NUM_RIGID_PARAMS
void rigid_transform_to_array(Eigen::Affine3d const& aff, double* arr);

// Convert an array of length NUM_RIGID_PARAMS to a rigid
// transform. Normalize the quaternion to make it into a rotation.
void array_to_rigid_transform(Eigen::Affine3d& aff, const double* arr);

void affine_transform_to_array(Eigen::Affine3d const& aff, double* arr);
void array_to_affine_transform(Eigen::Affine3d& aff, const double* arr);

// Convert a string of values separated by spaces to a vector of doubles.
std::vector<double> string_to_vector(std::string const& str);

// Read a 4x4 pose matrix of doubles from disk
void readPoseMatrix(cv::Mat& pose, std::string const& filename);

// Read an affine matrix with double values
bool readAffine(Eigen::Affine3d& T, std::string const& filename);

// Write a matrix with double values
void writeMatrix(Eigen::MatrixXd const& M, std::string const& filename);

void writeCloud(std::vector<float> const& points, size_t point_size, std::string const& filename);

// Return the type of an opencv matrix
std::string matType(cv::Mat const& mat);

// Read the transform from depth to given camera
void readCameraTransform(config_reader::ConfigReader& config, std::string const transform_str,
                         Eigen::Affine3d& transform);

// Read some transforms from the robot calibration file
void readConfigFile                                                     // NOLINT
(// Inputs                                                              // NOLINT
 std::vector<std::string> const& cam_names,                             // NOLINT
 std::string const& nav_cam_to_body_trans_str,                          // NOLINT
 std::string const& haz_cam_depth_to_image_trans_str,                   // NOLINT
 // Outputs                                                             // NOLINT
 std::vector<camera::CameraParameters> & cam_params,                    // NOLINT
 std::vector<Eigen::Affine3d>          & nav_to_cam_trans,              // NOLINT
 std::vector<double>                   & nav_to_cam_timestamp_offset,   // NOLINT
 Eigen::Affine3d                       & nav_cam_to_body_trans,         // NOLINT
 Eigen::Affine3d                       & haz_cam_depth_to_image_trans); // NOLINT

// Save some transforms from the robot calibration file. This has some very fragile
// logic and cannot handle comments in the config file.
void updateConfigFile                                                           // NOLINT
(std::vector<std::string>              const& cam_names,                        // NOLINT
 std::string                           const& haz_cam_depth_to_image_trans_str, // NOLINT
 std::vector<camera::CameraParameters> const& cam_params,                       // NOLINT
 std::vector<Eigen::Affine3d>          const& nav_to_cam_trans,                 // NOLINT
 std::vector<double>                   const& nav_to_cam_timestamp_offset,      // NOLINT
 Eigen::Affine3d                       const& haz_cam_depth_to_image_trans);    // NOLINT

// Given two poses aff0 and aff1, and 0 <= alpha <= 1, do linear interpolation.
Eigen::Affine3d linearInterp(double alpha, Eigen::Affine3d const& aff0,
                               Eigen::Affine3d const& aff1);

// Given a set of poses indexed by timestamp in an std::map, find the
// interpolated pose at desired timestamp. This is efficient
// only for very small maps. Else use the StampedPoseStorage class.
bool findInterpPose(double desired_time, std::map<double, Eigen::Affine3d> const& poses,
                    Eigen::Affine3d& interp_pose);

// A class to store timestamped poses, implementing O(log(n)) linear
// interpolation at a desired timestamp. For fast access, keep the
// poses in bins obtained by flooring the timestamp, which is measured
// in seconds. It is assumed that there are a handful of poses
// measured every second, so in each bin. When bins get large, or too
// many bins are empty, the efficiency of this algorithm goes down.
class StampedPoseStorage {
 public:
  void addPose(Eigen::Affine3d const& pose, double timestamp);

  // Find the interpolated pose by looking up the two poses with
  // closest timestamps that are below and above input_timestamp. If
  // the gap between those timestamps is more than max_gap, return
  // failure, as then likely the interpolation result is not accurate.
  bool interpPose(double input_timestamp, double max_gap, Eigen::Affine3d& out_pose) const;

  void clear();

  bool empty() const;

 private:
  std::map<int, std::map<double, Eigen::Affine3d> > m_poses;
};

// Compute the azimuth and elevation for a (normal) vector
void normalToAzimuthAndElevation(Eigen::Vector3d const& normal, double& azimuth, double& elevation);

// Compute a normal vector based on the azimuth and elevation angles
void azimuthAndElevationToNormal(Eigen::Vector3d& normal, double azimuth, double elevation);

// Snap the normal to the plane (and the plane itself) to make
// all angles multiple of 45 degrees with the coordinate axes.
void snapPlaneNormal(Eigen::Vector3d& plane_normal);

// Find the best fitting plane to a set of points
void bestFitPlane(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& centroid, Eigen::Vector3d& plane_normal);

// Extract from a string of the form someDir/1234.5678.jpg the number 123.456.
double fileNameToTimestamp(std::string const& file_name);

// Create a directory unless it exists already
void createDir(std::string const& dir);

// A little holding structure for nav, sci, and haz poses
struct CameraPoses {
  std::map<double, double> haz_depth_to_image_timestamps;
  std::map<std::string, std::map<double, Eigen::Affine3d> > world_to_cam_poses;
};

// Some small utilities for writing a file having poses for nav, sci, and haz cam,
// and also the depth timestamp corresponding to given haz intensity timestamp
void writeCameraPoses(std::string const& filename, std::map<double, double> const& haz_depth_to_image_timestamps,
                      std::map<std::string, std::map<double, Eigen::Affine3d> > const& world_to_cam_poses);

void readCameraPoses(std::string const& filename, std::map<double, double>& haz_depth_to_image_timestamps,
                     std::map<std::string, std::map<double, Eigen::Affine3d> >& world_to_cam_poses);

// Gamma and inverse gamma functions
// https://en.wikipedia.org/wiki/SRGB#Specification_of_the_transformation
double gamma(double x);
double inv_gamma(double x);

// Apply the inverse gamma transform to images, multiply them by
// max_iso_times_exposure/ISO/exposure_time to adjust for
// lightning differences, then apply the gamma transform back.
void exposureCorrection(double max_iso_times_exposure, double iso, double exposure, cv::Mat const& input_image,
                        cv::Mat& output_image);

// Scale an image to correct for lightning variations by taking into
// account that JPEG images have gamma correction applied to them.
// See https://en.wikipedia.org/wiki/Gamma_correction.
void scaleImage(double max_iso_times_exposure, double iso, double exposure, cv::Mat const& input_image,
                cv::Mat& output_image);

// Given two bounds, pick two timestamps within these bounds, the one
// closest to the left bound and the one to the right bound. Take into
// account that the timestamps may need to have an offset added to
// them. Assume that the input timestamps are sorted in increasing order.
// TODO(oalexan1): May have to add a constraint to only pick
// a timestamp if not further from the bound than a given value.
void pickTimestampsInBounds(std::vector<double> const& timestamps, double left_bound, double right_bound, double offset,
                            std::vector<double>& out_timestamps);

// Must always have NUM_EXIF the last.
enum ExifData { TIMESTAMP = 0, EXPOSURE_TIME, ISO, APERTURE, FOCAL_LENGTH, NUM_EXIF };

// Triangulate two rays emanating from given undistorted and centered pixels
Eigen::Vector3d TriangulatePair(double focal_length1, double focal_length2, Eigen::Affine3d const& world_to_cam1,
                                Eigen::Affine3d const& world_to_cam2, Eigen::Vector2d const& pix1,
                                Eigen::Vector2d const& pix2);

// Triangulate n rays emanating from given undistorted and centered pixels
Eigen::Vector3d Triangulate(std::vector<double> const& focal_length_vec,
                            std::vector<Eigen::Affine3d> const& world_to_cam_vec,
                            std::vector<Eigen::Vector2d> const& pix_vec);

// A utility for saving a camera in a format ASP understands.
// TODO(oalexan1): Expose the sci cam intrinsics rather than having
// them hard-coded.
void save_tsai_camera(Eigen::MatrixXd const& desired_cam_to_world_trans,
                      std::string const& output_dir,
                      double curr_time, std::string const& suffix);

}  // namespace dense_map

#endif  // DENSE_MAP_UTILS_H_
