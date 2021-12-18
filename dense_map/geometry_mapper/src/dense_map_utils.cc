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

#include <glog/logging.h>
#include <ff_common/thread.h>
#include <ff_common/utils.h>
#include <msg_conversions/msg_conversions.h>

#include <dense_map_utils.h>

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

#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>

namespace dense_map {

// A  function to split a string like 'optical_center focal_length' into
// its two constituents.
void parse_intrinsics_to_float(std::string const& intrinsics_to_float, std::set<std::string>& intrinsics_to_float_set) {
  intrinsics_to_float_set.clear();
  std::string val;
  std::istringstream is(intrinsics_to_float);
  while (is >> val) {
    if (val != "focal_length" && val != "optical_center" && val != "distortion")
      LOG(FATAL) << "Unexpected intrinsic name: " << val << std::endl;
    intrinsics_to_float_set.insert(val);
  }
}

// Extract a rigid transform to an array of length NUM_RIGID_PARAMS
void rigid_transform_to_array(Eigen::Affine3d const& aff, double* arr) {
  for (size_t it = 0; it < 3; it++) arr[it] = aff.translation()[it];

  Eigen::Quaterniond R(aff.linear());
  arr[3] = R.x();
  arr[4] = R.y();
  arr[5] = R.z();
  arr[6] = R.w();
}

// Convert an array of length NUM_RIGID_PARAMS to a rigid
// transform. Normalize the quaternion to make it into a rotation.
void array_to_rigid_transform(Eigen::Affine3d& aff, const double* arr) {
  for (size_t it = 0; it < 3; it++) aff.translation()[it] = arr[it];

  Eigen::Quaterniond R(arr[6], arr[3], arr[4], arr[5]);
  R.normalize();

  aff = Eigen::Affine3d(Eigen::Translation3d(arr[0], arr[1], arr[2])) * Eigen::Affine3d(R);
}

// Read a 4x4 pose matrix of doubles from disk
void readPoseMatrix(cv::Mat& pose, std::string const& filename) {
  pose = cv::Mat::zeros(4, 4, CV_64F);
  std::ifstream ifs(filename.c_str(), std::ifstream::in);
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      double val;
      if (!(ifs >> val)) LOG(FATAL) << "Could not read a 4x4 matrix from: " << filename;
      pose.at<double>(row, col) = val;
    }
  }
}

// Read an affine matrix with double values
bool readAffine(Eigen::Affine3d& T, std::string const& filename) {
  Eigen::MatrixXd M(4, 4);

  std::ifstream ifs(filename.c_str(), std::ifstream::in);
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      double val;
      if (!(ifs >> val)) return false;

      M(row, col) = val;
    }
  }

  T.linear() = M.block<3, 3>(0, 0);
  T.translation() = M.block<3, 1>(0, 3);

  return true;
}

// Write a matrix with double values
void writeMatrix(Eigen::MatrixXd const& M, std::string const& filename) {
  std::cout << "Writing: " << filename << std::endl;
  std::ofstream ofs(filename.c_str());
  ofs.precision(17);
  ofs << M << "\n";
  ofs.close();
}

// Save a file with x, y, z rows if point_size is 3, and also a color
// if point_size is 4.
void writeCloud(std::vector<float> const& points, size_t point_size, std::string const& filename) {
  size_t num_points = points.size() / point_size;
  if (point_size * num_points != points.size()) LOG(FATAL) << "Book-keeping failure.";

  std::cout << "Writing: " << filename << "\n";
  std::ofstream fh(filename.c_str());
  fh.precision(17);
  for (size_t it = 0; it < num_points; it++) {
    for (size_t ch = 0; ch < point_size; ch++) {
      fh << points[point_size * it + ch];
      if (ch + 1 < point_size)
        fh << " ";
      else
        fh << "\n";
    }
  }
  fh.close();
}

// Return the type of an opencv matrix
std::string matType(cv::Mat const& mat) {
  int type = mat.type();
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

// Read the transform from depth to given camera
void readCameraTransform(config_reader::ConfigReader& config, std::string const transform_str,
                         Eigen::Affine3d& transform) {
  Eigen::Vector3d T;
  Eigen::Quaterniond R;
  if (!msg_conversions::config_read_transform(&config, transform_str.c_str(), &T, &R))
    LOG(FATAL) << "Unspecified transform: " << transform_str << " for robot: "
               << getenv("ASTROBEE_ROBOT") << "\n";

  R.normalize();

  transform = Eigen::Affine3d(Eigen::Translation3d(T.x(), T.y(), T.z())) * Eigen::Affine3d(R);
}

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
 Eigen::Affine3d                       & haz_cam_depth_to_image_trans){ // NOLINT
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("transforms.config");
  if (!config.ReadFiles()) LOG(FATAL) << "Failed to read config files.";

  cam_params.clear();
  nav_to_cam_trans.clear();
  nav_to_cam_timestamp_offset.clear();
  for (size_t it = 0; it < cam_names.size(); it++) {
    camera::CameraParameters params = camera::CameraParameters(&config, cam_names[it].c_str());
    cam_params.push_back(params);

    std::string trans_str = "nav_cam_to_" + cam_names[it] + "_transform";
    if (cam_names[it] == "nav_cam") {
      // transforms from nav cam to itself
      nav_to_cam_trans.push_back(Eigen::Affine3d::Identity());
      nav_to_cam_timestamp_offset.push_back(0.0);
    } else {
      Eigen::Affine3d trans;
      readCameraTransform(config, trans_str, trans);
      nav_to_cam_trans.push_back(trans);

      std::string offset_str = "nav_cam_to_" + cam_names[it] + "_timestamp_offset";
      double offset;
      if (!config.GetReal(offset_str.c_str(), &offset))
        LOG(FATAL) << "Could not read value of " << offset_str
                   << " for robot: " << getenv("ASTROBEE_ROBOT");
      nav_to_cam_timestamp_offset.push_back(offset);
    }
  }

  // Read the remaining data
  readCameraTransform(config, nav_cam_to_body_trans_str, nav_cam_to_body_trans);

  Eigen::MatrixXd M(4, 4);
  config_reader::ConfigReader::Table mat(&config, haz_cam_depth_to_image_trans_str.c_str());
  int count = 0;
  for (int row = 0; row < M.rows(); row++) {
    for (int col = 0; col < M.cols(); col++) {
      count++;  // note that the count stats from 1
      if (!mat.GetReal(count, &M(row, col))) {
        LOG(FATAL) << "Could not read value of " << haz_cam_depth_to_image_trans_str
                   << " for robot: " << getenv("ASTROBEE_ROBOT");
      }
    }
  }

  haz_cam_depth_to_image_trans.matrix() = M;
}

// Given two poses aff0 and aff1, and 0 <= alpha <= 1, do linear interpolation.
Eigen::Affine3d linearInterp(double alpha, Eigen::Affine3d const& aff0, Eigen::Affine3d const& aff1) {
  Eigen::Quaternion<double> rot0(aff0.linear());
  Eigen::Quaternion<double> rot1(aff1.linear());

  Eigen::Vector3d trans0 = aff0.translation();
  Eigen::Vector3d trans1 = aff1.translation();

  Eigen::Affine3d result;

  result.translation() = (1.0 - alpha) * trans0 + alpha * trans1;
  result.linear() = rot0.slerp(alpha, rot1).toRotationMatrix();

  return result;
}

// Given a set of poses indexed by timestamp in an std::map, find the
// interpolated pose at desired timestamp. This is efficient
// only for very small maps. Else use the StampedPoseStorage class.
bool findInterpPose(double desired_time, std::map<double, Eigen::Affine3d> const& poses, Eigen::Affine3d& interp_pose) {
  double left_time = std::numeric_limits<double>::max();
  double right_time = -left_time;
  for (auto it = poses.begin(); it != poses.end(); it++) {
    double curr_time = it->first;
    if (curr_time <= desired_time) {
      left_time = curr_time;  // this can only increase
    }
    if (curr_time >= desired_time) {
      // Here an "if" was used rather than "else", to be able to
      // handle the case when left_time == curr_time == right_time.
      right_time = curr_time;
      break;  // just passed the desired time, can stop now
    }
  }

  if (left_time > right_time) {
    // Could not bracket the desired time
    return false;
  }

  double alpha = (desired_time - left_time) / (right_time - left_time);
  if (left_time == right_time) alpha = 0.0;  // handle division by 0
  interp_pose = linearInterp(alpha, poses.find(left_time)->second, poses.find(right_time)->second);
  return true;
}

void StampedPoseStorage::addPose(Eigen::Affine3d const& pose, double timestamp) {
  int bin_index = floor(timestamp);
  m_poses[bin_index][timestamp] = pose;
}

bool StampedPoseStorage::interpPose(double input_timestamp, double max_gap, Eigen::Affine3d& out_pose) const {
  bool is_success = false;

  if (m_poses.empty()) return is_success;

  // Look for the nearest pose with timestamp <= input_timestamp.
  double low_timestamp = -1.0;
  Eigen::Affine3d low_pose;
  // Traverse the bins in decreasing order of bin key.
  for (int bin_iter = floor(input_timestamp); bin_iter >= m_poses.begin()->first; bin_iter--) {
    auto bin_ptr = m_poses.find(bin_iter);
    if (bin_ptr == m_poses.end()) continue;  // empty bin

    // Found a bin. Study it in decreasing order of timestamps.
    auto& bin = bin_ptr->second;
    for (auto it = bin.rbegin(); it != bin.rend(); it++) {
      double timestamp = it->first;
      if (timestamp <= input_timestamp) {
        low_timestamp = timestamp;
        low_pose = it->second;
        is_success = true;
        break;
      }
    }
    if (is_success) break;
  }

  if (!is_success) return false;  // Failed

  // Found the lower bound. Now go forward in time. Here the logic is
  // the reverse of the above.
  is_success = false;
  double high_timestamp = -1.0;
  Eigen::Affine3d high_pose;
  for (int bin_iter = floor(input_timestamp); bin_iter <= m_poses.rbegin()->first; bin_iter++) {
    auto bin_ptr = m_poses.find(bin_iter);
    if (bin_ptr == m_poses.end()) continue;  // empty bin

    // Found a bin. Study it in increasing order of timestamps.
    auto& bin = bin_ptr->second;
    for (auto it = bin.begin(); it != bin.end(); it++) {
      double timestamp = it->first;
      if (timestamp >= input_timestamp) {
        high_timestamp = timestamp;
        high_pose = it->second;
        is_success = true;
        break;
      }
    }
    if (is_success) break;
  }

  if (!is_success || high_timestamp - low_timestamp > max_gap) {
    return false;  // Failed
  }

  if (!(low_timestamp <= input_timestamp && input_timestamp <= high_timestamp))
    LOG(FATAL) << "Book-keeping failure in pose interpolation.";

  double alpha = (input_timestamp - low_timestamp) / (high_timestamp - low_timestamp);
  if (high_timestamp == low_timestamp) alpha = 0.0;  // handle division by zero

  out_pose = dense_map::linearInterp(alpha, low_pose, high_pose);

  return is_success;
}

void StampedPoseStorage::clear() { m_poses.clear(); }

bool StampedPoseStorage::empty() const { return m_poses.empty(); }

// Compute the azimuth and elevation for a (normal) vector
void normalToAzimuthAndElevation(Eigen::Vector3d const& normal, double& azimuth, double& elevation) {
  if (normal.x() == 0 && normal.y() == 0) {
    azimuth = 0.0;
    if (normal.z() >= 0.0)
      elevation = M_PI / 2.0;
    else
      elevation = -M_PI / 2.0;
  } else {
    azimuth = atan2(normal.y(), normal.x());
    elevation = atan2(normal.z(), Eigen::Vector2d(normal.x(), normal.y()).norm());
  }
}

// Compute a normal vector based on the azimuth and elevation angles
void azimuthAndElevationToNormal(Eigen::Vector3d& normal, double azimuth, double elevation) {
  double ca = cos(azimuth), sa = sin(azimuth);
  double ce = cos(elevation), se = sin(elevation);
  normal = Eigen::Vector3d(ca * ce, sa * ce, se);
}

// Snap the normal to the plane (and the plane itself) to make
// all angles multiple of 45 degrees with the coordinate axes.
void snapPlaneNormal(Eigen::Vector3d& plane_normal) {
  double azimuth, elevation;
  normalToAzimuthAndElevation(plane_normal, azimuth, elevation);

  // Snap to multiple of 45 degrees
  double radian45 = M_PI / 4.0;
  azimuth = radian45 * round(azimuth / radian45);
  elevation = radian45 * round(elevation / radian45);

  azimuthAndElevationToNormal(plane_normal, azimuth, elevation);
}

// Find the best fitting plane to a set of points
void bestFitPlane(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& centroid,
                  Eigen::Vector3d& plane_normal) {
  // Copy coordinates to  matrix in Eigen format
  size_t num_points = points.size();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_points);

  for (size_t i = 0; i < num_points; i++) coord.col(i) = points[i];

  // calculate centroid
  centroid = Eigen::Vector3d(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

  // subtract centroid
  for (size_t it = 0; it < 3; it++) coord.row(it).array() -= centroid(it);

  // We only need the left-singular matrix here
  // https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  plane_normal = svd.matrixU().rightCols<1>();
}

// Extract from a string of the form someDir/1234.5678.jpg the number 123.456.
double fileNameToTimestamp(std::string const& file_name) {
  size_t beg = file_name.rfind("/");
  size_t end = file_name.rfind(".");
  if (beg == std::string::npos || end == std::string::npos || beg > end) {
    std::cout << "Could not parse file name: " + file_name;
    exit(1);
  }

  std::string frameStr = file_name.substr(beg + 1, end - beg - 1);
  return atof(frameStr.c_str());
}

// Create a directory unless it exists already
void createDir(std::string const& dir) {
  if (!boost::filesystem::create_directories(dir) && !boost::filesystem::exists(dir)) {
    LOG(FATAL) << "Failed to create directory: " << dir;
  }
}

// Minor utilities for converting values to a string below

std::string number_to_string(double val) {
  std::ostringstream oss;
  oss.precision(8);
  oss << val;
  return oss.str();
}

std::string vector_to_string(Eigen::VectorXd v) {
  std::ostringstream oss;
  oss.precision(8);
  oss << "{";
  for (int it = 0; it + 1 < v.size(); it++) oss << v[it] << ", ";
  if (v.size() > 0) oss << v[v.size() - 1] << "}";
  return oss.str();
}

std::string intrinsics_to_string(Eigen::Vector2d const& f, Eigen::Vector2d const& o) {
  std::ostringstream oss;
  oss.precision(8);
  oss << "{\n"
      << "      " << f[0] << ", 0.0, " << o[0] << ",\n"
      << "      0.0, " << f[1] << ", " << o[1] << ",\n"
      << "      0.0, 0.0, 1.0\n"
      << "    }";
  return oss.str();
}

std::string affine_to_string(Eigen::Affine3d const& T) {
  std::ostringstream oss;
  oss.precision(8);
  Eigen::Quaterniond q = Eigen::Quaterniond(T.linear());
  Eigen::Vector3d t = T.translation();
  oss << "(vec3(" << t[0] << ", " << t[1] << ", " << t[2] << "), quat4(" << q.x() << ", " << q.y() << ", " << q.z()
      << ", " << q.w() << "))";
  return oss.str();
}

std::string matrix_to_string(Eigen::MatrixXd const& M) {
  std::ostringstream oss;
  oss.precision(8);
  oss << "{\n";
  for (int row = 0; row < M.rows(); row++) {
    oss << "    ";
    for (int col = 0; col < M.cols(); col++) {
      oss << M(row, col);
      if (row < M.rows() - 1 || col < M.cols() - 1) {
        oss << ",";
        if (col < M.cols() - 1) oss << " ";
      }
    }
    if (row < M.rows() - 1) oss << "\n";
  }

  oss << "}";
  return oss.str();
}

// Find in the given text the given value, followed by some spaces perhaps, and followed
// by the equal sign. Start at position beg. Return the position after the equal sign.
int robust_find(std::string const& text, std::string const& val, int beg) {
  // Need a while loop since there may be more than one candidate. Stop at the first.
  int text_len = text.size();
  while (beg < text_len) {
    beg = text.find(val, beg);
    if (beg == std::string::npos) return beg;

    // TODO(oalexan1): Must skip comments.  From this position must
    // read back towards the beginning of the current line and see if
    // the text "--" is encountered, which would mean that this
    // position is on a comment and hence the search must continue.

    beg += val.size();  // advance

    // Look for spaces and the equal sign
    bool found = true;
    for (int it = beg; it < text_len; it++) {
      if (text[it] == '=') {
        beg = it + 1;
        return beg;
      }

      // No equal sign yet. So must have spaces until then.
      if (text[it] != ' ' && text[it] != '\t' && text[it] != '\n' && text[it] != '\r') {
        found = false;
        beg = it;
        break;
      }
    }

    // No luck, will try again from the current position.
    if (!found) continue;
  }

  return beg;
}
// Replace a given parameter's value in the text. This is very fragile
// code, particularly it does not understand that text starting with
// "--" is a comment.
void replace_param_val(std::string const& param_name, std::string const& param_val,
                       std::string const& parent,
                       std::string const& beg_brace, std::string const& end_brace,
                       // The text to modify
                       std::string& text) {
  int beg = 0;

  // First find the parent, if provided
  if (parent != "") {
    beg = robust_find(text, parent, beg);
    if (beg == std::string::npos) LOG(FATAL) << "Could not find the field '" << parent << " =' in the config file.";
  }

  // Find the param after the parent
  beg = robust_find(text, param_name, beg);
  if (beg == std::string::npos) {
    std::string msg;
    if (parent != "") msg = " Tried to locate it after field '" + parent + "'.";
    LOG(FATAL) << "Could not find the field '" << param_name << " =' in the config file." << msg;
  }

  int end = beg + 1;
  if (beg_brace != "" && end_brace != "") {  // The text to replace is in braces
    if (beg_brace.size() != 1 && end_brace.size() != 1)
      LOG(FATAL) << "Expecting one character for each of " << beg_brace << " and " << end_brace;

    beg = text.find(beg_brace, beg);
    if (beg == std::string::npos) {
      LOG(FATAL) << "Failed to replace value for " << parent << " " << param_name << " in the config file.";
    }
    end = beg + 1;

    // Find the matching brace
    int count = 1;
    while (end < static_cast<int>(text.size())) {
      if (text[end] == beg_brace[0])
        count++;
      else if (text[end] == end_brace[0])
        count--;
      if (count == 0) break;

      end++;
    }

  } else {
    // No braces, then just look for the next comma
    end = text.find(",", beg);
    if (beg == std::string::npos) LOG(FATAL) << "Could not parse correctly " << param_name;
    end--;  // go before the comma
  }

  text = text.replace(beg, end - beg + 1, param_val);
}

// Save some transforms from the robot calibration file. This has some very fragile
// logic and cannot handle comments in the config file.
void updateConfigFile                                                           // NOLINT
(std::vector<std::string>              const& cam_names,                        // NOLINT
 std::string                           const& haz_cam_depth_to_image_trans_str, // NOLINT
 std::vector<camera::CameraParameters> const& cam_params,                       // NOLINT
 std::vector<Eigen::Affine3d>          const& nav_to_cam_trans,                 // NOLINT
 std::vector<double>                   const& nav_to_cam_timestamp_offset,      // NOLINT
 Eigen::Affine3d                       const& haz_cam_depth_to_image_trans) {   // NOLINT
  if (cam_names.size() != cam_params.size()  ||
      cam_names.size() != nav_to_cam_trans.size() ||
      cam_names.size() != nav_to_cam_timestamp_offset.size())
    LOG(FATAL) << "The number of various inputs to updateConfigFile() do not match.";

  // Open the config file to modify
  char* config_dir = getenv("ASTROBEE_CONFIG_DIR");
  if (config_dir == NULL) LOG(FATAL) << "The environmental variable ASTROBEE_CONFIG_DIR was not set.";
  char* robot = getenv("ASTROBEE_ROBOT");
  if (robot == NULL) LOG(FATAL) << "The environmental variable ASTROBEE_ROBOT was not set.";
  std::string config_file = std::string(config_dir) + "/robots/" + robot + ".config";
  std::ifstream ifs(config_file.c_str());
  if (!ifs.is_open()) LOG(FATAL) << "Could not open file: " << config_file;

  // Read its text in one string
  std::string text;
  std::string line;
  while (std::getline(ifs, line)) text += line + "\n";
  ifs.close();

  std::cout << "Updating: " << config_file << std::endl;

  for (size_t it = 0; it < cam_names.size(); it++) {
    std::string              const& cam_name = cam_names[it];  // alias
    camera::CameraParameters const& params = cam_params[it];   // alias

    std::string intrinsics = intrinsics_to_string(params.GetFocalVector(),
                                                  params.GetOpticalOffset());
    replace_param_val("intrinsic_matrix", intrinsics, cam_name, "{", "}", text);
    Eigen::VectorXd cam_distortion = params.GetDistortion();
    if (cam_distortion.size() > 1) {
      std::string distortion_str = vector_to_string(cam_distortion);
      replace_param_val("distortion_coeff", distortion_str, cam_name, "{", "}", text);
    } else if (cam_distortion.size() == 1) {
      std::string distortion_str = " " + number_to_string(cam_distortion[0]);
      replace_param_val("distortion_coeff", distortion_str, cam_name, "", "", text);
    } else {
      LOG(FATAL) << "Camera " << cam_name << " must have distortion.";
    }

    if (cam_names[it] == "nav_cam") continue;  // this will have the trivial transforms

    std::string trans_name = "nav_cam_to_" + cam_names[it] + "_transform";
    std::string trans_val = affine_to_string(nav_to_cam_trans[it]);
    replace_param_val(trans_name, trans_val, "", "(", ")", text);

    std::string offset_str = "nav_cam_to_" + cam_names[it] + "_timestamp_offset";
    replace_param_val(offset_str, " " + number_to_string(nav_to_cam_timestamp_offset[it]), "",
                        "", "", text);
  }

  std::string depth_to_image_transform_val
    = matrix_to_string(haz_cam_depth_to_image_trans.matrix());
  replace_param_val(haz_cam_depth_to_image_trans_str, depth_to_image_transform_val,
                    "", "{", "}", text);

  // Write the updated values
  std::ofstream ofs(config_file.c_str());
  ofs << text;
  ofs.close();
}

void readCameraPoses(std::string const& filename,
                     std::map<double, double>& haz_depth_to_image_timestamps,
                     std::map<std::string, std::map<double, Eigen::Affine3d> >&
                     world_to_cam_poses) {
  haz_depth_to_image_timestamps.clear();
  world_to_cam_poses.clear();

  std::ifstream ifs(filename.c_str());
  std::string line;
  while (std::getline(ifs, line)) {
    std::istringstream is(line);

    std::string str;
    if (!(is >> str)) continue;

    if (str == "nav_cam" || str == "sci_cam" || str == "haz_cam") {
      double timestamp;
      if (!(is >> timestamp)) continue;

      Eigen::MatrixXd M(4, 4);
      for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
          if (!(is >> M(row, col))) LOG(FATAL) << "Failure reading: " << filename;
        }
      }

      world_to_cam_poses[str][timestamp].matrix() = M;

    } else if (str == "haz_depth_to_image") {
      double depth_time, image_time;
      if (!(is >> depth_time >> image_time)) LOG(FATAL) << "Failure reading: " << filename;

      haz_depth_to_image_timestamps[depth_time] = image_time;
    }
  }

  ifs.close();
}

// Gamma correction for x between 0 and 1.
// https://en.wikipedia.org/wiki/SRGB#Specification_of_the_transformation
double gamma(double x) {
  // return pow(x, 1.0/2.6);

  if (x <= 0.0031308) return 12.92 * x;

  return 1.055 * pow(x, 1.0 / 2.4) - 0.055;
}

double inv_gamma(double x) {
  // return pow(x, 2.6);

  if (x <= 0.04045) return x / 12.92;

  return pow((x + 0.055) / (1.055), 2.4);
}

// Apply the inverse gamma transform to images, multiply them by
// max_iso_times_exposure/ISO/exposure_time to adjust for
// lightning differences, then apply the gamma transform back.
void exposureCorrection(double max_iso_times_exposure, double iso, double exposure, cv::Mat const& input_image,
                        cv::Mat& output_image) {
  double scale = max_iso_times_exposure / iso / exposure;

  // Make an image of the same type
  input_image.copyTo(output_image);

  // Apply the inverse gamma correction, multiply by scale,
  // and apply the correction back
#pragma omp parallel for
  for (int row = 0; row < input_image.rows; row++) {
    for (int col = 0; col < input_image.cols; col++) {
      cv::Vec3b b = input_image.at<cv::Vec3b>(row, col);

      cv::Vec3b c;
      for (int color = 0; color < 3; color++) {
        double x = 255.0 * gamma(inv_gamma(static_cast<double>(b[color]) / 255.0) * scale);
        c[color] = std::min(round(x), 255.0);
      }
      output_image.at<cv::Vec3b>(row, col) = c;
    }
  }
}

// Scale an image to correct for lightning variations by taking into
// account that JPEG images have gamma correction applied to them.
// See https://en.wikipedia.org/wiki/Gamma_correction.
void scaleImage(double max_iso_times_exposure, double iso, double exposure, cv::Mat const& input_image,
                cv::Mat& output_image) {
  double scale = pow(max_iso_times_exposure / iso / exposure, 1.0 / 2.2);
  int same_type = -1;
  double offset = 0.0;
  input_image.convertTo(output_image, same_type, scale, offset);
}

// Given two bounds, pick two timestamps within these bounds, the one
// closest to the left bound and the one to the right bound. Take into
// account that the timestamps may need to have an offset added to
// them. Assume that the input timestamps are sorted in increasing order.
// TODO(oalexan1): May have to add a constraint to only pick
// a timestamp if not further from the bound than a given value.
void pickTimestampsInBounds(std::vector<double> const& timestamps, double left_bound, double right_bound, double offset,
                            std::vector<double>& out_timestamps) {
  out_timestamps.clear();

  // Start by simply collecting all timestamps between the given
  // bounds. Much easier to understand than if doing something more
  // fancy.
  std::vector<double> local_timestamps;
  for (size_t it = 0; it < timestamps.size(); it++) {
    double timestamp = timestamps[it];
    if (timestamp + offset >= left_bound && timestamp + offset < right_bound) {
      local_timestamps.push_back(timestamp);
    }
  }

  if (local_timestamps.size() < 1) {
    // Nothing to pick
    return;
  }

  if (local_timestamps.size() == 1) {
    // Only one is present
    out_timestamps.push_back(local_timestamps[0]);
    return;
  }

  // Add the ones at the ends
  out_timestamps.push_back(local_timestamps[0]);
  out_timestamps.push_back(local_timestamps.back());

  return;
}

// Triangulate rays emanating from given undistorted and centered pixels
Eigen::Vector3d TriangulatePair(double focal_length1, double focal_length2,
                                Eigen::Affine3d const& world_to_cam1,
                                Eigen::Affine3d const& world_to_cam2,
                                Eigen::Vector2d const& pix1,
                                Eigen::Vector2d const& pix2) {
  Eigen::Matrix3d k1;
  k1 << focal_length1, 0, 0, 0, focal_length1, 0, 0, 0, 1;

  Eigen::Matrix3d k2;
  k2 << focal_length2, 0, 0, 0, focal_length2, 0, 0, 0, 1;

  openMVG::Mat34 cid_to_p1, cid_to_p2;
  openMVG::P_From_KRt(k1, world_to_cam1.linear(), world_to_cam1.translation(), &cid_to_p1);
  openMVG::P_From_KRt(k2, world_to_cam2.linear(), world_to_cam2.translation(), &cid_to_p2);

  openMVG::Triangulation tri;
  tri.add(cid_to_p1, pix1);
  tri.add(cid_to_p2, pix2);

  Eigen::Vector3d solution = tri.compute();
  return solution;
}

// Triangulate n rays emanating from given undistorted and centered pixels
Eigen::Vector3d Triangulate(std::vector<double>          const& focal_length_vec,
                            std::vector<Eigen::Affine3d> const& world_to_cam_vec,
                            std::vector<Eigen::Vector2d> const& pix_vec) {
  if (focal_length_vec.size() != world_to_cam_vec.size() ||
      focal_length_vec.size() != pix_vec.size())
    LOG(FATAL) << "All inputs to Triangulate() must have the same size.";

  if (focal_length_vec.size() <= 1)
    LOG(FATAL) << "At least two rays must be passed to Triangulate().";

  openMVG::Triangulation tri;

  for (size_t it = 0; it < focal_length_vec.size(); it++) {
    Eigen::Matrix3d k;
    k << focal_length_vec[it], 0, 0, 0, focal_length_vec[it], 0, 0, 0, 1;

    openMVG::Mat34 cid_to_p;
    openMVG::P_From_KRt(k, world_to_cam_vec[it].linear(), world_to_cam_vec[it].translation(),
                        &cid_to_p);

    tri.add(cid_to_p, pix_vec[it]);
  }

  Eigen::Vector3d solution = tri.compute();
  return solution;
}

// A debug utility for saving a camera in a format ASP understands.
// Need to expose the sci cam intrinsics.
void save_tsai_camera(Eigen::MatrixXd const& desired_cam_to_world_trans, std::string const& output_dir,
                      double curr_time, std::string const& suffix) {
  char filename_buffer[1000];
  auto T = desired_cam_to_world_trans;
  double shift = 6378137.0;  // planet radius, pretend this is a satellite camera
  snprintf(filename_buffer, sizeof(filename_buffer), "%s/%10.7f_%s.tsai", output_dir.c_str(), curr_time,
           suffix.c_str());
  std::cout << "Writing: " << filename_buffer << std::endl;
  std::ofstream ofs(filename_buffer);
  ofs.precision(18);
  ofs << "VERSION_3\n";
  ofs << "fu = 1138.4943\n";
  ofs << "fv = 1138.4943\n";
  ofs << "cu = 680.36447\n";
  ofs << "cv = 534.00133\n";
  ofs << "u_direction = 1 0 0\n";
  ofs << "v_direction = 0 1 0\n";
  ofs << "w_direction = 0 0 1\n";
  ofs << "C = " << T(0, 3) + shift << ' ' << T(1, 3) << ' ' << T(2, 3) << "\n";
  ofs << "R = " << T(0, 0) << ' ' << T(0, 1) << ' ' << T(0, 2) << ' ' << T(1, 0) << ' ' << T(1, 1) << ' ' << T(1, 2)
      << ' ' << T(2, 0) << ' ' << T(2, 1) << ' ' << T(2, 2) << "\n";
  ofs << "pitch = 1\n";
  ofs << "NULL\n";
  ofs.close();
}

}  // end namespace dense_map
