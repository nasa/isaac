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
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

// TODO(oalexan1): Modularize this code and move things to utils

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
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <boost/filesystem.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <ff_common/utils.h>
#include <ff_util/ff_names.h>
#include <sparse_mapping/sparse_map.h>

#include <dense_map_ros_utils.h>
#include <dense_map_utils.h>
#include <happly.h>  // for reading and writing ply files

#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <array>

// Read from the bag in intrinsics information for the given simulated camera save it to disk
void saveSimCamIntrinsics(std::string const& bag_file, std::string cam_type,
                          std::string const& output_dir) {
  int image_width, image_height;
  double focal_length, optical_center_x, optical_center_y;

  std::vector<std::string> topics;
  std::string info_topic = "/sim/" + cam_type + "/info";
  topics.push_back(info_topic);

  bool success = false;
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for (rosbag::MessageInstance const m : view) {
    sensor_msgs::CameraInfo::ConstPtr info_msg = m.instantiate<sensor_msgs::CameraInfo>();
    if (info_msg) {
      image_width = info_msg->width;
      image_height = info_msg->height;
      focal_length = info_msg->K[0];
      optical_center_x = info_msg->K[2];
      optical_center_y = info_msg->K[5];

      // Found what we wanted
      success = true;
      break;
    }
  }

  if (!success) {
    LOG(FATAL) << "Could not read intrinsics for simulated camera " << cam_type
               << " from bag: " << bag_file << " on topic: " << info_topic << "\n";
    return;
  }

  std::string intrinsics_file = output_dir + "/undistorted_intrinsics.txt";
  std::cout << "Writing: " << intrinsics_file << std::endl;
  std::ofstream ofs(intrinsics_file.c_str());
  if (!ofs.good()) {
    LOG(FATAL) << "Could not write: " << intrinsics_file;
    return;
  }
  ofs.precision(17);
  ofs << "# Unidistored width and height, focal length, undistorted optical center\n";
  ofs << image_width << ' ' << image_height << ' ' << focal_length << ' '
      << optical_center_x << ' ' << optical_center_y << "\n";
  ofs.close();
}

// Given a nav cam timestamp, find at most 4 images in the sparse map
// around that timestamp. Note that we use the counter sparse_id to
// travel forward in time through the sparse map upon repeated
// invocations of this function to narrow down the search.
bool findNearbySparseMapImages(double desired_nav_cam_time,
                               std::vector<double> const& sparse_map_timestamps,
                               int & sparse_map_id,  // gets modified
                               std::vector<int> & nearby_cid) {
  int num_cid = sparse_map_timestamps.size();
  if (num_cid < 2) LOG(FATAL) << "Must have a map with at least two images.";

  // Wipe the output
  nearby_cid.clear();

  // If we overstep the bounds of the sparse map timestamps (when
  // using the SURF map with the map made up of the same images that
  // we want to texture) localization may become unreliable.
  bool within_bounds =
    (sparse_map_timestamps[0] <= desired_nav_cam_time &&
     desired_nav_cam_time <= sparse_map_timestamps[num_cid - 1]);
  if (!within_bounds) return false;

  // Try to find this many images in the map around the time in desired_nav_cam_time.
  int num_similar = std::min(4, num_cid);

  // Find the largest image index in the map with timestamp <=
  // desired_nav_cam_time.  Note that we use the variable
  // sparse_map_id that is passed from outside that we update here.
  for (int cid = sparse_map_id; cid < num_cid; cid++) {
    if (sparse_map_timestamps[cid] <= desired_nav_cam_time) {
      sparse_map_id = cid;  // sparse_map_id can only increase
    } else {
      // Went too far
      break;
    }
  }

  // Pick the image frames in the sparse map that are closest in time,
  // both earlier and later.

  int num_found = 0;
  int left_cid = sparse_map_id, right_cid = sparse_map_id + 1;
  for (int attempt = 0; attempt < 2 * num_similar; attempt++) {
    if (left_cid >= 0) {  // ensure we don't go out of bounds
      nearby_cid.push_back(left_cid);
      left_cid--;
      num_found++;
    }
    if (num_found >= num_similar) break;

    if (right_cid < num_cid) {  // ensure we don't go out of bounds
      nearby_cid.push_back(right_cid);
      right_cid++;
      num_found++;
    }
    if (num_found >= num_similar) break;
  }

  // We found the cid by alternating between left and right, but now
  // put them in order.
  std::sort(nearby_cid.begin(), nearby_cid.end());

  return true;
}

// Given a timestamp of a desired camera (haz_cam or sci_cam) and the
// list of nav_cam image timestamps, find the nearest nav_cam images in
// time to the desired timestamp, localize from the sparse map their
// poses, then interpolate and transform to find the pose of the
// desired camera.
bool findInterpolatedPose(double desired_nav_cam_time,
                          std::vector<double> const& nav_cam_bag_timestamps,
                          Eigen::Affine3d const& nav_cam_to_desired_cam_trans,
                          std::vector<rosbag::MessageInstance> const& nav_cam_msgs,
                          boost::shared_ptr<sparse_mapping::SparseMap> sparse_map,
                          bool use_brisk_map,
                          std::vector<double> const& sparse_map_timestamps,
                          Eigen::MatrixXd& desired_cam_to_world_trans,
                          int& sparse_map_id, int& nav_cam_bag_id, int& nav_cam_pos) {
  // Find nav cam images around the nav cam pose to compute
  std::vector<int>* nearby_cid_ptr = NULL;
  std::vector<int> nearby_cid;
  if (!use_brisk_map) {
    if (!findNearbySparseMapImages(desired_nav_cam_time, sparse_map_timestamps,
                                   sparse_map_id,  // gets modified
                                   nearby_cid))
      return false;  // Failed to find images nearby
    nearby_cid_ptr = &nearby_cid;
  } else {
    // For a brisk map we will localize based on the vocab db, so we don't need
    // find nearby images.
  }

  // Find the two image timestamps that bracket a given cloud timestamp.
  // In particular, the last cloud timestamp will likely not be used
  // if there is no image timestamp after it.
  int num_nav_cam_timestamps = nav_cam_bag_timestamps.size();
  for (int image_it = nav_cam_bag_id; image_it < num_nav_cam_timestamps; image_it++) {
    double beg_nav_cam_time = nav_cam_bag_timestamps[image_it + 0];
    double end_nav_cam_time = nav_cam_bag_timestamps[image_it + 1];

    if (beg_nav_cam_time <= desired_nav_cam_time && desired_nav_cam_time < end_nav_cam_time) {
      nav_cam_bag_id = image_it;  // save this for next time, it can only increase

      // Localize the two images around the depth cloud image and then interpolate
      // between them.
      cv::Mat beg_image;
      bool save_grayscale = true;  // Localization needs grayscale images
      double found_time = -1.0;
      if (!dense_map::lookupImage(beg_nav_cam_time, nav_cam_msgs, save_grayscale,
                                  beg_image, nav_cam_pos, found_time))
        return false;

      camera::CameraModel beg_localized_cam(Eigen::Vector3d(), Eigen::Matrix3d::Identity(),
                                            sparse_map->GetCameraParameters());
      if (!sparse_map->Localize(beg_image, &beg_localized_cam, NULL, NULL, nearby_cid_ptr))
        return false;
      Eigen::Affine3d beg_trans = beg_localized_cam.GetTransform();

      // Note that below we use a throw-away value for nav_cam_pos.
      // That is because we would otherwise violate the assumption
      // that nav_cam_pos always goes forward in time. The position of
      // the end image now can be later in time than the position of
      // the beg image next time around.
      int end_nav_cam_pos = nav_cam_pos;
      cv::Mat end_image;
      if (!dense_map::lookupImage(end_nav_cam_time, nav_cam_msgs, save_grayscale, end_image,
                                  end_nav_cam_pos, found_time))
        return false;
      camera::CameraModel end_localized_cam(Eigen::Vector3d(), Eigen::Matrix3d::Identity(),
                                            sparse_map->GetCameraParameters());
      if (!sparse_map->Localize(end_image, &end_localized_cam, NULL, NULL, nearby_cid_ptr)) return false;
      Eigen::Affine3d end_trans = end_localized_cam.GetTransform();

      for (size_t it = 0; it < nearby_cid.size(); it++) {
        // std::cout << "nearby image: " << sparse_map->cid_to_filename_[nearby_cid[it]]
        //           << std::endl;
      }
      // std::cout << "localizing cloud at time " << desired_nav_cam_time << std::endl;
      double alpha = (desired_nav_cam_time - beg_nav_cam_time) / (end_nav_cam_time - beg_nav_cam_time);
      if (end_nav_cam_time == beg_nav_cam_time) alpha = 0.0;  // handle division by zero

      Eigen::Affine3d interp_trans = dense_map::linearInterp(alpha, beg_trans, end_trans);

      desired_cam_to_world_trans
        = interp_trans.inverse().matrix() * nav_cam_to_desired_cam_trans.matrix().inverse();

      return true;
    }
  }
  return false;
}

// Store x, y, z, intensity, and weight.
typedef cv::Vec<double, 5> Vec5d;

// Check if first three coordinates of a vector are 0
bool zeroXyz(Vec5d const& p) { return (p[0] == 0) && (p[1] == 0) && (p[2] == 0); }

// Add a given vertex to the ply file unless already present
inline void add_vertex(Vec5d const& V, std::pair<int, int> const& pix,        // NOLINT
                       std::map<std::pair<int, int>, size_t>& pix_to_vertex,  // NOLINT
                       size_t& vertex_count,                                  // NOLINT
                       std::vector<std::array<double, 3>>& vertices,          // NOLINT
                       std::vector<std::array<double, 3>>& colors) {          // NOLINT
  // Do not add the zero vertex
  if (zeroXyz(V)) return;

  if (pix_to_vertex.find(pix) != pix_to_vertex.end()) return;  // Vertex already exists

  std::array<double, 3> point = {V[0], V[1], V[2]};
  std::array<double, 3> color = {V[3] / 255.0, V[3] / 255.0, V[3] / 255.0};

  vertices.push_back(point);
  colors.push_back(color);

  // Record the map from the pixel to its location
  pix_to_vertex[pix] = vertex_count;
  vertex_count++;
}

double seg_len(Vec5d const& A, Vec5d const& B) {
  return Eigen::Vector3d(A[0] - B[0], A[1] - B[1], A[2] - B[2]).norm(); // NOLINT
}

void add_dist(Vec5d const& A, Vec5d const& B, std::vector<double>& distances) {
  if (!zeroXyz(A) && !zeroXyz(B)) distances.push_back(seg_len(A, B));
}

// If the distance between two points is more than this len,
// add more points in between on the edge connecting them.
void add_points(Vec5d const& A, Vec5d const& B, double min_len, std::vector<Vec5d>& points) {
  if (zeroXyz(A) || zeroXyz(B)) return;

  double curr_len = seg_len(A, B);
  if (curr_len <= min_len) return;

  // Total number of points including the endpoints
  int num = round(curr_len / min_len) + 1;

  // Insert points outside the endpoints. Note that the colors and
  // weights (positions 3 and 4) are also computed via the linear
  // interpolation.
  for (int i = 1; i < num; i++) {
    double t = static_cast<double>(i) / static_cast<double>(num);

    Vec5d C = (1.0 - t) * A + t * B;
    points.push_back(C);
  }
}

// Estimate the median mesh edge. Given for corners, all possible segments between them are
// possible edges.
double median_mesh_edge(cv::Mat const& depthMat) {
  std::vector<double> distances;

  for (int row = 0; row < depthMat.rows - 1; row++) {
    for (int col = 0; col < depthMat.cols - 1; col++) {
      std::pair<int, int> pix_ul = std::make_pair(row, col);
      std::pair<int, int> pix_ur = std::make_pair(row + 1, col);
      std::pair<int, int> pix_ll = std::make_pair(row, col + 1);
      std::pair<int, int> pix_lr = std::make_pair(row + 1, col + 1);
      Vec5d UL = depthMat.at<Vec5d>(pix_ul.first, pix_ul.second);
      Vec5d UR = depthMat.at<Vec5d>(pix_ur.first, pix_ur.second);
      Vec5d LL = depthMat.at<Vec5d>(pix_ll.first, pix_ll.second);
      Vec5d LR = depthMat.at<Vec5d>(pix_lr.first, pix_lr.second);

      add_dist(UL, UR, distances);
      add_dist(UL, LL, distances);
      add_dist(UL, LR, distances);
      add_dist(UR, LL, distances);
      add_dist(UR, LR, distances);
      add_dist(LL, LR, distances);
    }
  }

  if (distances.empty()) return 0.0;

  std::sort(distances.begin(), distances.end());
  double median = distances[distances.size() / 2];

  return median;
}

// Add points on edges and inside triangles having distances longer
// than the median edge length times a factor. Store the combined set
// in depthMat by adding further rows to it.
void add_extra_pts(cv::Mat& depthMat, cv::Mat& workMat, Vec5d const& Zero) {
  double median_len = median_mesh_edge(depthMat);

  if (median_len <= 0) return;  // should not happen

  // Add points if lengths are more than this
  double min_len = 1.3 * median_len;

  std::vector<Vec5d> points;

  for (int row = 0; row < depthMat.rows - 1; row++) {
    for (int col = 0; col < depthMat.cols - 1; col++) {
      std::pair<int, int> pix_ul = std::make_pair(row, col);
      std::pair<int, int> pix_ur = std::make_pair(row + 1, col);
      std::pair<int, int> pix_ll = std::make_pair(row, col + 1);
      std::pair<int, int> pix_lr = std::make_pair(row + 1, col + 1);
      Vec5d UL = depthMat.at<Vec5d>(pix_ul.first, pix_ul.second);
      Vec5d UR = depthMat.at<Vec5d>(pix_ur.first, pix_ur.second);
      Vec5d LL = depthMat.at<Vec5d>(pix_ll.first, pix_ll.second);
      Vec5d LR = depthMat.at<Vec5d>(pix_lr.first, pix_lr.second);

      // Add points on each edge
      std::vector<Vec5d> points1;
      add_points(UL, UR, min_len, points1);
      add_points(UL, LL, min_len, points1);
      add_points(UR, LL, min_len, points1);

      int curr_len1 = points1.size();

      if (!zeroXyz(UL) && !zeroXyz(UR) && !zeroXyz(LL) && UL[3] >= 0 && UR[3] >= 0 && LL[3] >= 0) {
        // Valid face

        // See if to add the triangle center
        Vec5d C = (UL + UR + LL) / 3.0;
        if (seg_len(UL, C) > min_len || seg_len(UR, C) > min_len || seg_len(LL, C) > min_len) points1.push_back(C);

        // Add points from triangle center to each point added so far
        for (int i = 0; i < curr_len1; i++) add_points(points1[i], C, min_len, points1);
      }

      // Add points on each edge of this triangle
      std::vector<Vec5d> points2;
      add_points(UR, LR, min_len, points2);
      add_points(UR, LL, min_len, points2);
      add_points(LR, LL, min_len, points2);

      if (!zeroXyz(UR) && !zeroXyz(LR) && !zeroXyz(LL) && UR[3] >= 0 && LR[3] >= 0 && LL[3] >= 0) {
        // Valid face

        // See if to add the triangle center
        int curr_len2 = points2.size();
        Vec5d C = (UR + LR + LL) / 3.0;
        if (seg_len(UR, C) > min_len || seg_len(LR, C) > min_len || seg_len(LL, C) > min_len) points2.push_back(C);

        // Add points from triangle center to each point added so far
        for (int i = 0; i < curr_len2; i++) add_points(points2[i], C, min_len, points2);
      }

      // Append from point1 and point2 to points
      for (size_t it = 0; it < points1.size(); it++) points.push_back(points1[it]);
      for (size_t it = 0; it < points2.size(); it++) points.push_back(points2[it]);

      // Add points on the diagonal we did not consider so far
      add_points(UL, LR, min_len, points);
    }
  }

  // See how many more rows to add to fit in the same matrix.
  // Adding 1 there is important, to ensure we have more room than
  // what we need, to ensure the count reaches the number of points
  // below.
  int num_points = points.size();
  int num_new_rows = ceil(static_cast<double>(num_points) / static_cast<double>(depthMat.cols)) + 1;

  // Save a copy of depthMat before modifying it
  depthMat.copyTo(workMat);

  // Resize depthMat, all info in it is now lost, and have only zeros
  depthMat = cv::Mat_<Vec5d>(workMat.rows + num_new_rows, workMat.cols, Zero);

  // Copy the relevant chunk from workMat
  for (int row = 0; row < workMat.rows; row++) {
    for (int col = 0; col < workMat.cols; col++) {
      depthMat.at<Vec5d>(row, col) = workMat.at<Vec5d>(row, col);
    }
  }

  // Add the extra points
  int count = -1;
  for (int row = workMat.rows; row < depthMat.rows; row++) {
    for (int col = 0; col < depthMat.cols; col++) {
      count++;

      if (count >= num_points) break;

      depthMat.at<Vec5d>(row, col) = points[count];
      depthMat.at<Vec5d>(row, col)[3] = -1;  // so that the mesh does not have it as a face
    }

    // Need to make sure to break the outer loop too
    if (count >= num_points) break;
  }

  if (count != num_points)
    LOG(FATAL) << "Bookkeeping error. Did not add all the points to the depth map.";
}

// Form a mesh. Ignore (x, y, z, i, w) values with (x, y, z) = (0, 0, 0).
// Add a vertex but not a face if i is negative.
void save_mesh(cv::Mat const& depthMat, std::string const& plyFileName) {
  size_t vertex_count = 0;
  std::map<std::pair<int, int>, size_t> pix_to_vertex;  // map from pixel to vertex indices
  std::vector<std::array<double, 3>> vertices;
  std::vector<std::array<double, 3>> colors;
  std::vector<std::vector<size_t>> faces;

  for (int row = 0; row < depthMat.rows - 1; row++) {
    for (int col = 0; col < depthMat.cols - 1; col++) {
      std::pair<int, int> pix_ul = std::make_pair(row, col);
      std::pair<int, int> pix_ur = std::make_pair(row + 1, col);
      std::pair<int, int> pix_ll = std::make_pair(row, col + 1);
      std::pair<int, int> pix_lr = std::make_pair(row + 1, col + 1);
      Vec5d UL = depthMat.at<Vec5d>(pix_ul.first, pix_ul.second);
      Vec5d UR = depthMat.at<Vec5d>(pix_ur.first, pix_ur.second);
      Vec5d LL = depthMat.at<Vec5d>(pix_ll.first, pix_ll.second);
      Vec5d LR = depthMat.at<Vec5d>(pix_lr.first, pix_lr.second);

      // Add a vertex even though no face has it as a corner
      add_vertex(UL, pix_ul, pix_to_vertex, vertex_count, vertices, colors);
      add_vertex(UR, pix_ur, pix_to_vertex, vertex_count, vertices, colors);
      add_vertex(LL, pix_ll, pix_to_vertex, vertex_count, vertices, colors);

      // Note how we add only valid faces. For the latter the vertices
      // must be nonzero and the last coordinate must be positive.
      if (!zeroXyz(UL) && !zeroXyz(UR) && !zeroXyz(LL) &&
          UL[3] >= 0 && UR[3] >= 0 && LL[3] >= 0) {
        std::vector<size_t> face = {pix_to_vertex[pix_ul], pix_to_vertex[pix_ur],
                                    pix_to_vertex[pix_ll]};
        faces.push_back(face);
      }

      // Add a vertex even though no face has it as a corner
      add_vertex(UR, pix_ur, pix_to_vertex, vertex_count, vertices, colors);
      add_vertex(LR, pix_lr, pix_to_vertex, vertex_count, vertices, colors);
      add_vertex(LL, pix_ll, pix_to_vertex, vertex_count, vertices, colors);

      // Note how we add only valid faces. For the latter the vertices
      // must be nonzero and the last coordinate must be positive.
      if (!zeroXyz(UR) && !zeroXyz(LR) && !zeroXyz(LL) &&
          UR[3] >= 0 && LR[3] >= 0 && LL[3] >= 0) {
        std::vector<size_t> face = {pix_to_vertex[pix_ur], pix_to_vertex[pix_lr],
                                    pix_to_vertex[pix_ll]};
        faces.push_back(face);
      }
    }
  }

  // Form and save the ply
  happly::PLYData ply;
  ply.addVertexPositions(vertices);
  ply.addVertexColors(colors);
  ply.addFaceIndices(faces);
  std::cout << "Writing: " << plyFileName << std::endl;
  ply.write(plyFileName, happly::DataFormat::ASCII);
}

// Find the median of a sorted vector
// TODO(oalexan1): May need to put here a partial check that the vector is sorted
double find_median(std::vector<double> const& v) {
  if (v.empty()) LOG(FATAL) << "Cannot find the median of an empty vector.\n";

  int a = (v.size() - 1) / 2;
  int b = (v.size() - 0) / 2;

  return (v[a] + v[b]) / 2.0;
}

// Throw out as outliers points whose x, y, or z values differs
// from the median of such values by more than this distance.
// This can be expensive for win >= 25.
void median_filter(cv::Mat& depthMat, cv::Mat& workMat, Vec5d const& Zero, int win, double delta) {
  // Copy it to workMat, with the latter not being modified below
  depthMat.copyTo(workMat);

  for (int row = 0; row < workMat.rows; row++) {
#pragma omp parallel for
    for (int col = 0; col < workMat.cols; col++) {
      std::vector<double> dist_x, dist_y, dist_z;

      Vec5d pt = workMat.at<Vec5d>(row, col);

      // Is an outlier already
      if (zeroXyz(pt)) continue;

      for (int irow = -win / 2; irow <= win / 2; irow++) {
        for (int icol = -win / 2; icol <= win / 2; icol++) {
          // Skip if out of bounds
          if (row + irow < 0 || row + irow >= workMat.rows) continue;
          if (col + icol < 0 || col + icol >= workMat.cols) continue;

          Vec5d curr_pt = workMat.at<Vec5d>(row + irow, col + icol);

          if (zeroXyz(curr_pt)) continue;

          dist_x.push_back(curr_pt[0]);
          dist_y.push_back(curr_pt[1]);
          dist_z.push_back(curr_pt[2]);
        }
      }

      if (dist_x.size() < std::max(2, std::min(5, win))) {
        // so few neighbors, could just toss it out
        depthMat.at<Vec5d>(row, col) = Zero;
        continue;
      }

      std::sort(dist_x.begin(), dist_x.end());
      std::sort(dist_y.begin(), dist_y.end());
      std::sort(dist_z.begin(), dist_z.end());

      if (std::abs(find_median(dist_x) - pt[0]) > delta || std::abs(find_median(dist_y) - pt[1]) > delta ||
          std::abs(find_median(dist_z) - pt[2]) > delta) {
        depthMat.at<Vec5d>(row, col) = Zero;
      }
    }
  }
}

// Hole-fill a pixel unless it results in steep drops.  A very small
// foreshortening_delta means a new introduced point will result in a
// ray from the existing point to the new point being almost parallel
// to existing nearby rays (emanating from camera center) which is not
// good.
void hole_fill(cv::Mat& depthMat, cv::Mat& workMat, double sigma, Vec5d const& Zero, int radius1, int radius2,
               double foreshortening_delta) {
  // Copy depthMat to workMat, with the latter unchanged below
  depthMat.copyTo(workMat);

  for (int row = 0; row < depthMat.rows; row++) {
#pragma omp parallel for
    for (int col = 0; col < depthMat.cols; col++) {
      // Points which do not need to be filled can be skipped
      // and let them keep their existing value
      if (!zeroXyz(workMat.at<Vec5d>(row, col))) {
        depthMat.at<Vec5d>(row, col) = workMat.at<Vec5d>(row, col);
        continue;
      }

      // How many valid pixels are in each quadrant.
      // A point on an axis counts towards both quadrants it touches.
      cv::Mat valid(cv::Size(2, 2), CV_8U, cv::Scalar(0));

      Vec5d pt_sum = Zero;
      double wt_sum = 0.0;

      for (int irow = -radius1; irow <= radius1; irow++) {
        for (int icol = -radius1; icol <= radius1; icol++) {
          // Skip if out of bounds
          if (row + irow < 0 || row + irow >= depthMat.rows) continue;
          if (col + icol < 0 || col + icol >= depthMat.cols) continue;

          double dist = sqrt(irow * irow * 1.0 + icol * icol * 1.0);

          // Look only within given radius
          if (dist > static_cast<double>(radius1)) continue;

          Vec5d curr_pt = workMat.at<Vec5d>(row + irow, col + icol);

          // Note that these are not exclusive, as points on axes
          // can be in multiple quadrants
          if (irow >= 0 && icol >= 0) {
            // First quadrant
            int pos_row = 1, pos_col = 1;
            if (!zeroXyz(curr_pt)) valid.at<uchar>(pos_row, pos_col) = 1;
          }
          if (irow >= 0 && icol <= 0) {
            // Second quadrant
            int pos_row = 1, pos_col = 0;
            if (!zeroXyz(curr_pt)) valid.at<uchar>(pos_row, pos_col) = 1;
          }
          if (irow <= 0 && icol <= 0) {
            // Third quadrant
            int pos_row = 0, pos_col = 0;
            if (!zeroXyz(curr_pt)) valid.at<uchar>(pos_row, pos_col) = 1;
          }
          if (irow <= 0 && icol >= 0) {
            // fourth quadrant
            int pos_row = 0, pos_col = 1;
            if (!zeroXyz(curr_pt)) valid.at<uchar>(pos_row, pos_col) = 1;
          }

          if (zeroXyz(curr_pt)) continue;  // Not a valid point

          double wt = exp(-std::min(sigma * dist * dist, 200.0));  // avoid underflow
          if (wt == 0.0) continue;

          // Form the weighted average
          pt_sum += wt * curr_pt;
          wt_sum += wt;
        }
      }

      // Accept this point only if there are enough points to average in
      // all four quadrants
      bool will_accept = true;
      int num_good_quadrants = 0;
      for (int irow = 0; irow < 2; irow++) {
        for (int icol = 0; icol < 2; icol++) {
          if (valid.at<uchar>(irow, icol) > 0) num_good_quadrants++;
        }
      }

      if (num_good_quadrants < 4) will_accept = false;

      if (!will_accept || wt_sum == 0.0) {
        depthMat.at<Vec5d>(row, col) = Zero;
        continue;
      }

      Vec5d new_pt = pt_sum / wt_sum;
      Eigen::Vector3d Z(0, 0, 1);

      // See if this point results in rays which are too parallel to existing rays
      for (int irow = -radius2; irow <= radius2; irow++) {
        for (int icol = -radius2; icol <= radius2; icol++) {
          // Skip if out of bounds
          if (row + irow < 0 || row + irow >= depthMat.rows) continue;
          if (col + icol < 0 || col + icol >= depthMat.cols) continue;

          double dist = sqrt(irow * irow * 1.0 + icol * icol * 1.0);

          // Check only within given radius
          if (dist > static_cast<double>(radius2)) continue;

          Vec5d curr_pt = workMat.at<Vec5d>(row + irow, col + icol);

          if (zeroXyz(curr_pt)) continue;  // Not a valid point

          // Direction measured from center of camera
          Eigen::Vector3d curr_dir(curr_pt[0], curr_pt[1], curr_pt[2]);

          // Direction from current point to new point we may add
          Eigen::Vector3d new_dir = Eigen::Vector3d(new_pt[0], new_pt[1], new_pt[2]) - curr_dir;

          // Also handle the case when this angle may be invalid
          double cos_angle = new_dir.dot(curr_dir) / (curr_dir.norm() * new_dir.norm());
          double angle = (180.0 / M_PI) * acos(cos_angle);
          if (angle < foreshortening_delta || 180.0 - angle < foreshortening_delta ||
              std::isnan(angle) || std::isinf(angle)) {
            will_accept = false;
            break;
          }
        }

        if (!will_accept) break;
      }

      if (!will_accept) {
        depthMat.at<Vec5d>(row, col) = Zero;
        continue;
      }

      // accept
      depthMat.at<Vec5d>(row, col) = new_pt;
    }
  }
}

// Smooth newly added points and also their immediate neighbors
void smooth_additions(cv::Mat const& origMat, cv::Mat& depthMat, cv::Mat& workMat,
                      double sigma, Vec5d const& Zero, int radius) {
  // Copy depthMat to workMat, with the latter not being changed below.
  // Note how we use origMat to keep track of points which did not
  // change recently.
  depthMat.copyTo(workMat);

  for (int row = 0; row < depthMat.rows; row++) {
#pragma omp parallel for
    for (int col = 0; col < depthMat.cols; col++) {
      // Points which are not new or their neighbors are left untouched
      bool isNew = false;
      for (int irow = -1; irow <= 1; irow++) {
        for (int icol = -1; icol <= 1; icol++) {
          // Skip if out of bounds
          if (row + irow < 0 || row + irow >= depthMat.rows) continue;
          if (col + icol < 0 || col + icol >= depthMat.cols) continue;

          if (zeroXyz(origMat.at<Vec5d>(row + irow, col + icol))) {
            isNew = true;
            break;
          }
        }
        if (isNew) break;
      }

      if (!isNew) continue;

      // Do not smooth points which have no good value to start with
      if (zeroXyz(workMat.at<Vec5d>(row, col))) continue;

      Vec5d pt_sum = Zero;
      double wt_sum = 0.0;

      for (int irow = -radius; irow <= radius; irow++) {
        for (int icol = -radius; icol <= radius; icol++) {
          // Skip if out of bounds
          if (row + irow < 0 || row + irow >= depthMat.rows) continue;
          if (col + icol < 0 || col + icol >= depthMat.cols) continue;

          double dist = sqrt(irow * irow * 1.0 + icol * icol * 1.0);

          if (dist > static_cast<double>(radius)) continue;

          Vec5d curr_pt = workMat.at<Vec5d>(row + irow, col + icol);

          if (zeroXyz(curr_pt)) continue;  // Not a valid point

          double wt = exp(-std::min(sigma * dist * dist, 200.0));  // avoid underflow
          if (wt == 0.0) continue;

          // Form the weighted average
          pt_sum += wt * curr_pt;
          wt_sum += wt;
        }
      }

      Vec5d new_pt = pt_sum / wt_sum;
      depthMat.at<Vec5d>(row, col) = new_pt;
    }
  }
}

// Add weights which are higher at image center and decrease towards
// boundary. This makes voxblox blend better.
void calc_weights(cv::Mat& depthMat, double exponent, bool simulated_data) {
  for (int row = 0; row < depthMat.rows; row++) {
#pragma omp parallel for
    for (int col = 0; col < depthMat.cols; col++) {
      if (zeroXyz(depthMat.at<Vec5d>(row, col))) continue;

      double drow = row - depthMat.rows / 2.0;
      double dcol = col - depthMat.cols / 2.0;
      double dist_sq = drow * drow + dcol * dcol;

      // For sim data use a weight of 1.0 for each point
      float weight = 1.0;
      if (!simulated_data) {
        // Some kind of function decaying from center
        weight = 1.0 / pow(1.0 + dist_sq, exponent/2.0);

        // Also, points that are further in z are given less weight
        double z = depthMat.at<Vec5d>(row, col)[2];
        weight *= 1.0 / (0.001 + z * z);

        // Make sure the weight does not get too small as voxblox
        // will read it as a float. Here making the weights
        // just a little bigger than what voxblox will accept.
        weight = std::max(weight, static_cast<float>(1.1e-6));
      }

      depthMat.at<Vec5d>(row, col)[4] = weight;
    }
  }
}

void save_proc_depth_cloud(sensor_msgs::PointCloud2::ConstPtr pc_msg,
                           cv::Mat const& haz_cam_intensity,
                           Eigen::Affine3d const& haz_cam_depth_to_image_transform,
                           bool simulated_data, int depth_exclude_columns, int depth_exclude_rows,
                           double foreshortening_delta, double depth_hole_fill_diameter,
                           double reliability_weight_exponent,
                           std::vector<double> const& median_filter_params, bool save_debug_data,
                           const char* filename_buffer,
                           Eigen::MatrixXd const& desired_cam_to_world_trans) {
  // Sanity checks
  if (static_cast<int>(haz_cam_intensity.cols) != static_cast<int>(pc_msg->width) ||
      static_cast<int>(haz_cam_intensity.rows) != static_cast<int>(pc_msg->height))
    LOG(FATAL) << "Depth image dimensions does not agree with point cloud dimensions.";
  if (dense_map::matType(haz_cam_intensity) != "8UC3")
    LOG(FATAL) << "Extracted depth image should be of type 8UC3.";

  // Extract the depth point cloud from the message
  pcl::PointCloud<pcl::PointXYZ> pc;
  dense_map::msgToPcl(pc_msg, pc);
  if (static_cast<int>(pc.points.size()) != static_cast<int>(pc_msg->width * pc_msg->height))
    LOG(FATAL) << "Extracted point cloud size does not agree with original size.";

  // Two constants
  float inf = std::numeric_limits<float>::infinity();
  Vec5d Zero(0, 0, 0, 0, 0);

  // A matrix storing the depth cloud and a temporary work matrix.
  // These have 5 channels: x, y, z, intensity, and weight. The weight
  // determines the reliability of a point.
  cv::Mat_<Vec5d> depthMat(haz_cam_intensity.rows, haz_cam_intensity.cols, Zero);
  cv::Mat_<Vec5d> workMat(haz_cam_intensity.rows, haz_cam_intensity.cols, Zero);

  // Populate depthMat
  int count = -1;
  for (int row = 0; row < depthMat.rows; row++) {
    for (int col = 0; col < depthMat.cols; col++) {
      count++;
      double x = pc.points[count].x;
      double y = pc.points[count].y;
      double z = pc.points[count].z;

      // Pick first channel. The precise color of the depth cloud is not important.
      // It will be wiped later during smoothing and hole-filling anyway.
      double i = haz_cam_intensity.at<cv::Vec3b>(row, col)[0];
      bool skip = (col < depth_exclude_columns || depthMat.cols - col < depth_exclude_columns ||
                   row < depth_exclude_rows || depthMat.rows - row < depth_exclude_rows);

      if ((x == 0 && y == 0 && z == 0) || skip)
        depthMat.at<Vec5d>(row, col) = Zero;
      else
        depthMat.at<Vec5d>(row, col) = Vec5d(x, y, z, i, 0);
    }
  }

  // Throw out as outliers points whose x, y, or z values differs
  // from the median of such values by more than given distance.
  // This can be a slow operation for big windows.
  if (!simulated_data) {
    int num_filter_passes = median_filter_params.size() / 2;
    for (int pass = 0; pass < num_filter_passes; pass++)
      median_filter(depthMat, workMat, Zero, median_filter_params[2 * pass + 0],
                    median_filter_params[2 * pass + 1]);
  }

  // Make a copy of the current depth image before filling holes
  cv::Mat origMat;
  depthMat.copyTo(origMat);

  // Fill holes using these radii
  int radius1 = depth_hole_fill_diameter / 2.0;
  // Use a smaller radius for the foreshortening constraint
  int radius2 = depth_hole_fill_diameter / 4.0;
  double sigma = 0.5;  // gaussian sigma used for hole-filling and smoothing
  if (depth_hole_fill_diameter > 0 && !simulated_data)
    hole_fill(depthMat, workMat, sigma, Zero, radius1, radius2, foreshortening_delta);

  // Smooth newly added points and also their immediate neighbors.
  // Use origMat for reference.
  int radius = depth_hole_fill_diameter / 4.0;
  if (depth_hole_fill_diameter > 0 && !simulated_data)
    smooth_additions(origMat, depthMat, workMat, sigma, Zero, radius);

  // This is the right place at which to add weights, before adding
  // extra points messes up with the number of rows in in depthMat.
  calc_weights(depthMat, reliability_weight_exponent, simulated_data);

  // Add extra points, but only if we are committed to manipulating the
  // depth cloud to start with
  if (depth_hole_fill_diameter > 0 && !simulated_data)
    add_extra_pts(depthMat, workMat, Zero);

  if (save_debug_data) {
    // Save the updated depthMat as a mesh (for debugging)
    std::string plyFileName = filename_buffer + std::string(".ply");
    save_mesh(depthMat, plyFileName);
  }

  // Initialize point cloud that we will pass to voxblox
  pcl::PointCloud<pcl::PointNormal> pci;
  pci.width = depthMat.cols;
  pci.height = depthMat.rows;
  pci.points.resize(pci.width * pci.height);

  count = -1;
  for (int row = 0; row < depthMat.rows; row++) {
    for (int col = 0; col < depthMat.cols; col++) {
      count++;

      Vec5d pt = depthMat.at<Vec5d>(row, col);

      // An outlier
      if (zeroXyz(pt)) {
        // This is needed for VoxBlox.
        pci.points[count].x = inf;
        pci.points[count].y = inf;
        pci.points[count].z = inf;
        pci.points[count].normal_x = 0;   // intensity
        pci.points[count].normal_y = 0;   // weight
        pci.points[count].normal_z = 0;   // ensure initialization
        pci.points[count].curvature = 0;  // ensure initialization
        continue;
      }

      Eigen::Vector3d X(pt[0], pt[1], pt[2]);
      X = haz_cam_depth_to_image_transform * X;
      pci.points[count].x = X[0];
      pci.points[count].y = X[1];
      pci.points[count].z = X[2];

      if (pt[3] <= 0) pt[3] = 100;  // Ensure a positive value for the color
      if (pt[4] <  0) pt[4] = 0;    // Ensure a non-negative weight

      pci.points[count].normal_x = pt[3];  // intensity
      pci.points[count].normal_y = pt[4];  // weight
      pci.points[count].normal_z  = 0;     // ensure initialization
      pci.points[count].curvature = 0;     // ensure initialization
    }
  }

  // Save the pcd file
  std::cout << "Writing: " << filename_buffer << std::endl;
  pcl::io::savePCDFileASCII(filename_buffer, pci);

  if (save_debug_data) {
    // Find the image of distances in depthMat, for debugging. Note
    // that we ignore the extra points we added as new rows depthMat in
    // add_extra_pts().

    // The depthMat may have extra rows, but we assume the number of columns
    // did not change.
    if (haz_cam_intensity.cols != depthMat.cols)
      LOG(FATAL) << "Incorrect number of columns in depthMat\n";

    cv::Mat depthDist(haz_cam_intensity.rows, haz_cam_intensity.cols, CV_32FC1, cv::Scalar(0));
    count = -1;
    for (int row = 0; row < depthDist.rows; row++) {
      for (int col = 0; col < depthDist.cols; col++) {
        Vec5d pt = depthMat.at<Vec5d>(row, col);
        Eigen::Vector3d eigen_pt(pt[0], pt[1], pt[2]);
        double dist = eigen_pt.norm();
        depthDist.at<float>(row, col) = dist;
      }
    }

    // Find the valid range of distances
    double min_dist = std::numeric_limits<double>::max();
    double max_dist = -min_dist;
    for (int row = 0; row < depthDist.rows; row++) {
      for (int col = 0; col < depthDist.cols; col++) {
        double dist = depthDist.at<float>(row, col);
        if (dist <= 0) continue;

        min_dist = std::min(min_dist, dist);
        max_dist = std::max(max_dist, dist);
      }
    }

    if (min_dist > max_dist) {
      // No valid samples
      min_dist = 0.0;
      max_dist = 1.0;
    }

    if (min_dist == max_dist) max_dist = min_dist + 1.0;  // just one sample

    // Write the normalized file
    std::string fileName = filename_buffer + std::string(".png");
    std::cout << "Writing: " << fileName << std::endl;
    cv::imwrite(fileName, 255.0 * (depthDist - min_dist) / (max_dist - min_dist));

    // Transform (modifies depthMat) and save the transformed mesh. For debugging.
    for (int row = 0; row < depthMat.rows; row++) {
      for (int col = 0; col < depthMat.cols; col++) {
        Vec5d pt = depthMat.at<Vec5d>(row, col);

        if (zeroXyz(pt))
          continue;

        // Apply an affine transform
        Eigen::Vector3d X(pt[0], pt[1], pt[2]);
        X = haz_cam_depth_to_image_transform * X;

        // Apply a 4x4 rotation + translation transform
        Eigen::VectorXd V(4);
        V << X[0], X[1], X[2], 1;
        V = desired_cam_to_world_trans * V;

        for (size_t it = 0; it < 3; it++)
          depthMat.at<Vec5d>(row, col)[it] = V[it];
      }
    }

    std::string plyFileName = filename_buffer + std::string("_trans.ply");
    save_mesh(depthMat, plyFileName);
  }
}

// Given a desired camera and a set of acquisition timestamps for it,
// find the nav cam images nearest in time bracketing each timestamp,
// find the poses of those images using localization, interpolate the
// nav cam pose at the desired timestamp, and apply the transform
// between the nav cam and the desired camera to find the pose of the
// desired camera (by which it is meant the transform from the desired
// camera to the world).

// Also write the extracted point clouds and the desired cam (nav or sci) images.

void saveCameraPoses(
  bool simulated_data, std::string const& cam_type, double nav_to_desired_cam_offset,
  Eigen::Affine3d const& nav_cam_to_desired_cam_trans,
  Eigen::Affine3d& haz_cam_depth_to_image_transform, int depth_exclude_columns,
  int depth_exclude_rows, double foreshortening_delta, double depth_hole_fill_diameter,
  double reliability_weight_exponent, std::vector<double> const& median_filter_params,
  std::vector<rosbag::MessageInstance> const& desired_cam_msgs,
  std::vector<rosbag::MessageInstance> const& nav_cam_msgs,  // always needed when not doing sim
  std::vector<rosbag::MessageInstance> const& haz_cam_points_msgs,
  std::vector<double> const& nav_cam_bag_timestamps,
  std::map<double, std::vector<double>> const& sci_cam_exif, std::string const& output_dir,
  std::string const& desired_cam_dir, double start, double duration,
  double sampling_spacing_seconds, double dist_between_processed_cams,
  std::set<double> const& sci_cam_timestamps, double max_iso_times_exposure,
  boost::shared_ptr<sparse_mapping::SparseMap> sparse_map, bool use_brisk_map,
  bool do_haz_cam_image,
  dense_map::StampedPoseStorage const& sim_desired_cam_poses, bool save_debug_data) {
  double min_map_timestamp = std::numeric_limits<double>::max();
  double max_map_timestamp = -min_map_timestamp;
  std::vector<double> sparse_map_timestamps;
  if (!simulated_data && !use_brisk_map) {
    // Find the minimum and maximum timestamps in the sparse map
    const std::vector<std::string>& images = sparse_map->cid_to_filename_;
    sparse_map_timestamps.resize(images.size());
    for (size_t cid = 0; cid < images.size(); cid++) {
      double timestamp = dense_map::fileNameToTimestamp(images[cid]);
      sparse_map_timestamps[cid] = timestamp;
      min_map_timestamp = std::min(min_map_timestamp, sparse_map_timestamps[cid]);
      max_map_timestamp = std::max(max_map_timestamp, sparse_map_timestamps[cid]);
    }
  }

  // Open a handle for writing the index of files being written to
  // disk.  This is not done if we only want to process the haz cam
  // cloud without the haz_cam image.
  std::string index_file = output_dir + "/" + cam_type + "_index.txt";
  std::ofstream ofs(index_file.c_str());

  // For the haz cam, need an image index, which will be haz_cam_index.txt,
  // and a point cloud index, which will be depth_cam_index.txt.
  std::string depth_index_file = output_dir + "/" + "depth_cam" + "_index.txt";
  std::ofstream depth_ofs;
  if (cam_type == "haz_cam") depth_ofs = std::ofstream(depth_index_file.c_str());

  // Keep track of where in the bags we are.
  int desired_cam_pos = 0;  // can be either nav, sci, or haz cam intensity position
  int nav_cam_pos = 0;      // track separately the nav cam position as well

  // These two are camera ids in the map used for different
  // purposes. We increment them as we travel forward in time.
  int sparse_map_id = 0, nav_cam_bag_id = 0;

  // These are used to track time as the bag is traversed
  double beg_time = -1.0, prev_time = -1.0;

  // This is used to ensure cameras are not too close
  Eigen::Vector3d prev_cam_ctr(std::numeric_limits<double>::quiet_NaN(), 0, 0);

  // Compute the starting bag time as the timestamp of the first cloud
  for (auto& m : haz_cam_points_msgs) {
    if (beg_time < 0) {
      sensor_msgs::PointCloud2::ConstPtr pc_msg;
      pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (pc_msg) {
        beg_time = pc_msg->header.stamp.toSec();
        break;
      }
    }
  }

  std::vector<rosbag::MessageInstance> const* iter_msgs = NULL;
  if (cam_type == "haz_cam")
    iter_msgs = &haz_cam_points_msgs;  // iterate over the point clouds
  else
    iter_msgs = &desired_cam_msgs;     // iterate over desired images

  for (auto& m : *iter_msgs) {
    sensor_msgs::PointCloud2::ConstPtr pc_msg;
    double curr_time = -1.0;

    if (cam_type == "haz_cam") {
      // Find the depth cloud and its time
      pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (!pc_msg) continue;
      curr_time = pc_msg->header.stamp.toSec();
    } else {
      // Find the image and its time
      sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
      if (image_msg) {
        curr_time = image_msg->header.stamp.toSec();
      } else {
        // Check for compressed image
        sensor_msgs::CompressedImage::ConstPtr comp_image_msg
          = m.instantiate<sensor_msgs::CompressedImage>();
        if (comp_image_msg) {
          curr_time = comp_image_msg->header.stamp.toSec();
        } else {
          // Not an image or compressed image topic
          continue;
        }
      }
    }

    bool custom_sci_timestamps = (cam_type == "sci_cam" && !sci_cam_timestamps.empty());
    if (!simulated_data && !use_brisk_map && !custom_sci_timestamps) {
      // Do not process data before the sparse map starts
      if (curr_time < min_map_timestamp) continue;
      // Do not process data after the sparse map ends
      if (curr_time > max_map_timestamp) break;
    }

    if (curr_time - beg_time < start) continue;  // did not yet reach the desired starting time

    if (duration > 0 && curr_time - beg_time > start + duration)
      break;  // past the desired ending time

    // The pose we will solve for
    bool lookup_success = false;
    Eigen::MatrixXd desired_cam_to_world_trans;

    // Let some time elapse between fusing clouds
    if (std::abs(curr_time - prev_time) < sampling_spacing_seconds && !custom_sci_timestamps) {
      continue;
    }

    // The case of custom timestamps
    if (custom_sci_timestamps && sci_cam_timestamps.find(curr_time) == sci_cam_timestamps.end())
      continue;

    if (!simulated_data) {
      // Do localization based on nav cam, transform the pose to desired cam coordinates, etc
      // Note that we convert from the desired cam time to nav cam time for this operation
      if (!findInterpolatedPose(curr_time - nav_to_desired_cam_offset,
                                nav_cam_bag_timestamps,
                                nav_cam_to_desired_cam_trans, nav_cam_msgs, sparse_map,
                                use_brisk_map,
                                sparse_map_timestamps, desired_cam_to_world_trans,
                                sparse_map_id, nav_cam_bag_id,
                                nav_cam_pos)) {
        std::cout << std::setprecision(17)
                  << "Localization failed at time " << curr_time << ". "
                  << "If too many such failures, consider modifying the value of "
                  << "--localization_options. See the documentation for more info.\n";
        continue;
      }
    } else {
      // In this case the logic is much simpler. We already have the
      // desired camera poses.  Yet, the poses and images for a given
      // camera need not be acquired at the same time, even in
      // simulation, so, do interpolation.
      Eigen::Affine3d out_pose;
      double max_gap = 1.0e+10;  // hoping the simulator is functioning well
      if (!sim_desired_cam_poses.interpPose(curr_time, max_gap, out_pose)) continue;
      desired_cam_to_world_trans = out_pose.matrix();
    }

    // Success localizing. Update the time at which it took place for next time.
    prev_time = curr_time;

    std::cout << "Time elapsed since the bag started: " << curr_time - beg_time << "\n";

    // See if to skip some images if the robot did not move much
    Eigen::Affine3d curr_trans;
    curr_trans.matrix() = desired_cam_to_world_trans;
    Eigen::Vector3d curr_cam_ctr = curr_trans.translation();
    double curr_dist_bw_cams = (prev_cam_ctr - curr_cam_ctr).norm();
    if (!std::isnan(prev_cam_ctr[0]) && !custom_sci_timestamps &&
        curr_dist_bw_cams < dist_between_processed_cams) {
      std::cout << "Distance to previously processed camera is "
                << curr_dist_bw_cams << ", skipping it."
                << std::endl;
      continue;
    }

    char filename_buffer[1000];
    std::string suffix = cam_type + "_to_world";

    // Lookup and save the image at the current time or soon after it (the latter
    // case happens when looking up the image based on the cloud time)
    cv::Mat desired_image;
    bool save_grayscale = false;  // Get a color image, if possible
    double found_time = -1.0;
    if (cam_type != "haz_cam" || do_haz_cam_image) {
      if (!dense_map::lookupImage(curr_time, desired_cam_msgs, save_grayscale, desired_image,
                                  desired_cam_pos, found_time))
        continue;  // the expected image could not be found
    } else {
      // We have cam_type == haz_cam but no haz_cam image texturing
      // desired. Create a fake image, only for the purpose of saving
      // the point cloud, for which an image is needed.
      desired_image = cv::Mat(pc_msg->height, pc_msg->width, CV_8UC3, cv::Scalar(255, 255, 255));
    }

    // Apply an optional scale. Use a pointer to not copy the data more than
    // one has to.
    cv::Mat* img_ptr = &desired_image;
    cv::Mat scaled_image;
    if (!simulated_data && cam_type == "sci_cam") {
      auto exif_it = sci_cam_exif.find(curr_time);
      if (exif_it != sci_cam_exif.end()) {
        std::vector<double> const& exif = exif_it->second;
        double iso = exif[dense_map::ISO];
        double exposure = exif[dense_map::EXPOSURE_TIME];
        dense_map::exposureCorrection(max_iso_times_exposure, iso, exposure,
                                      *img_ptr, scaled_image);
        img_ptr = &scaled_image;
      }
    }

    snprintf(filename_buffer, sizeof(filename_buffer), "%s/%10.7f.jpg",
             desired_cam_dir.c_str(), curr_time);
    std::cout << "Writing: " << filename_buffer << std::endl;
    cv::imwrite(filename_buffer, *img_ptr);

    // Save the name of the image in the index
    ofs << filename_buffer << "\n";

    // Save the transform
    snprintf(filename_buffer, sizeof(filename_buffer), "%s/%10.7f_%s.txt",
             output_dir.c_str(), curr_time, suffix.c_str());
    dense_map::writeMatrix(desired_cam_to_world_trans, filename_buffer);

    if (cam_type == "haz_cam") {
      // Save the transform name in the index file, for voxblox
      depth_ofs << filename_buffer << "\n";

      // Append the intensity to the point cloud and save it
      snprintf(filename_buffer, sizeof(filename_buffer), "%s/%10.7f.pcd",
               output_dir.c_str(), curr_time);
      save_proc_depth_cloud(pc_msg, desired_image, haz_cam_depth_to_image_transform,
                            simulated_data,
                            depth_exclude_columns, depth_exclude_rows,
                            foreshortening_delta, depth_hole_fill_diameter,
                            reliability_weight_exponent, median_filter_params,
                            save_debug_data, filename_buffer,
                            desired_cam_to_world_trans);

      // Save the name of the point cloud to the index
      depth_ofs << filename_buffer << "\n";
    }

    // Update the previous camera center
    prev_cam_ctr = curr_cam_ctr;
  }

  std::cout << "Wrote: " << index_file << std::endl;
}

void save_nav_cam_poses_and_images(boost::shared_ptr<sparse_mapping::SparseMap> sparse_map,
                                  std::vector<rosbag::MessageInstance> const& nav_cam_msgs,
                                  std::string const& output_dir,
                                  std::string const& nav_cam_dir) {
  // Keep track of where in the bag we are
  int nav_cam_pos = 0;

  std::string index_file = output_dir + "/nav_cam_index.txt";

  std::ofstream ofs(index_file.c_str());

  char filename_buffer[1000];

  for (unsigned int cid = 0; cid < sparse_map->cid_to_filename_.size(); cid++) {
    std::string img_file = sparse_map->cid_to_filename_[cid];

    double timestamp = dense_map::fileNameToTimestamp(img_file);

    std::string ext = ff_common::file_extension(img_file);
    std::string cam_file = ff_common::ReplaceInStr(img_file, "." + ext, "_nav_cam_to_world.txt");
    cam_file = output_dir + "/" + ff_common::basename(cam_file);
    if (cam_file == img_file) LOG(FATAL) << "Failed to replace the image extension in: "
                                         << img_file;

    dense_map::writeMatrix(sparse_map->cid_to_cam_t_global_[cid].inverse().matrix(), cam_file);

    cv::Mat nav_cam_image;
    bool save_grayscale = false;  // read color images, if found
    double found_time = -1.0;
    if (!dense_map::lookupImage(timestamp, nav_cam_msgs, save_grayscale, nav_cam_image,
                                nav_cam_pos, found_time))
      continue;

    snprintf(filename_buffer, sizeof(filename_buffer), "%s/%10.7f.jpg",
             nav_cam_dir.c_str(), timestamp);
    std::cout << "Writing: " << filename_buffer << std::endl;
    cv::imwrite(filename_buffer, nav_cam_image);

    ofs << filename_buffer << "\n";
  }
}

DEFINE_string(ros_bag, "", "A ROS bag with recorded nav_cam, haz_cam, and sci_cam data.");
DEFINE_string(sparse_map, "", "A registered sparse map made with some of the ROS bag data.");
DEFINE_string(output_dir, "", "The full path to a directory where to write the processed data.");
DEFINE_string(sci_cam_exif_topic, "/hw/sci_cam_exif", "The sci cam exif metadata topic the output bag.");
DEFINE_string(camera_types, "sci_cam nav_cam haz_cam",
              "Specify the cameras to use for the textures, as a list in quotes.");
DEFINE_string(camera_topics, "/hw/cam_sci/compressed /mgt/img_sampler/nav_cam/image_record "
              "/hw/depth_haz/extended/amplitude_int",
              "Specify the bag topics for the cameras to texture (in the same order as in "
              "--camera_types). Use a list in quotes.");
DEFINE_string(haz_cam_points_topic, "/hw/depth_haz/points",
              "The depth point cloud topic in the bag file.");
DEFINE_double(start, 0.0, "How many seconds into the bag to start processing the data.");
DEFINE_double(duration, -1.0, "For how many seconds to do the processing.");
DEFINE_double(sampling_spacing_seconds, 0.5,
              "Spacing to use, in seconds, between consecutive depth images in "
              "the bag that are processed.");
DEFINE_double(dist_between_processed_cams, 0.1,
              "Once an image or depth image is processed, how far the camera "
              "should move (in meters) before it should process more data.");
DEFINE_string(sci_cam_timestamps, "",
              "Process only these sci cam timestamps (rather than "
              "any in the bag using --dist_between_processed_cams, etc.). Must be "
              "a file with one timestamp per line.");
DEFINE_double(max_iso_times_exposure, 5.1,
              "Apply the inverse gamma transform to "
              "images, multiply them by max_iso_times_exposure/ISO/exposure_time "
              "to adjust for lightning differences, then apply the gamma "
              "transform back. This value should be set to the maximum observed "
              "ISO * exposure_time. The default is 5.1. Not used with simulated data.");
DEFINE_int32(depth_exclude_columns, 0,
             "Remove this many columns of data from the rectangular depth image sensor "
             "at margins to avoid some distortion of that data.");
DEFINE_int32(depth_exclude_rows, 0,
             "Remove this many rows of data from the rectangular depth image sensor "
             "at margins to avoid some distortion of that data.");
DEFINE_double(foreshortening_delta, 5.0,
              "A smaller value here will result in holes in depth images being filled more "
              "aggressively but potentially with more artifacts in foreshortened regions.");
DEFINE_string(median_filters, "7 0.1 25 0.1",
              "Given a list 'w1 d1 w2 d2 ... ', remove a depth image point "
              "if it differs, in the Manhattan norm, from the median of cloud points "
              "in the pixel window of size wi centered at it by more than di. This "
              "removes points sticking out for each such i.");
DEFINE_double(depth_hole_fill_diameter, 30.0,
              "Fill holes in the depth point clouds with this diameter, in pixels. This happens before the clouds "
              "are fused. It is suggested to not make this too big, as more hole-filling happens on the fused mesh "
              "later (--max_hole_diameter).");
DEFINE_double(reliability_weight_exponent, 2.0,
              "A larger value will give more weight to depth points corresponding to "
              "pixels closer to depth image center, which are considered more reliable.");
DEFINE_bool(simulated_data, false,
            "If specified, use data recorded in simulation. "
            "Then haz and sci camera poses and intrinsics should be recorded in the bag file.");
DEFINE_bool(use_brisk_map, false,
            "If specified, instead of a SURF sparse map made from the same bag that needs "
            "texturing, use a pre-existing and unrelated BRISK map. This map may be more "
            "convenient but less reliable.");
DEFINE_string(nav_cam_to_sci_cam_offset_override_value, "",
              "Override the value of nav_cam_to_sci_cam_timestamp_offset from the robot config "
              "file with this value.");
DEFINE_bool(save_debug_data, false, "Save many intermediate datasets for debugging.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (FLAGS_ros_bag.empty()) LOG(FATAL) << "The bag file was not specified.";
  if (FLAGS_output_dir.empty()) LOG(FATAL) << "The output directory was not specified.";
  if (FLAGS_sparse_map.empty() && !FLAGS_simulated_data)
    LOG(FATAL) << "The sparse map was not specified.";

  if (!boost::filesystem::exists(FLAGS_ros_bag)) LOG(FATAL) << "Bag does not exist: "
                                                            << FLAGS_ros_bag;

  if (!boost::filesystem::exists(FLAGS_sparse_map) && !FLAGS_simulated_data)
    LOG(FATAL) << "Sparse map does not exist: " << FLAGS_sparse_map;

  // Parse the median filter params
  double val;
  std::vector<double> median_filter_params;
  std::istringstream ifsm(FLAGS_median_filters);
  while (ifsm >> val) median_filter_params.push_back(val);
  if (median_filter_params.size() % 2 != 0)
    LOG(FATAL) << "There must exist an even number of median filter parameters, "
               << "being pairs of window sizes and distances.";

  // Parse the camera types
  std::vector<std::string> cam_types;
  std::string str;
  std::istringstream ifsc(FLAGS_camera_types);
  bool do_haz_cam_image = false, do_nav_cam = false;
  while (ifsc >> str) {
    cam_types.push_back(str);
    if (str == "haz_cam") do_haz_cam_image = true;
    if (str == "nav_cam") do_nav_cam = true;
  }
  if (!FLAGS_simulated_data && !do_nav_cam)
    LOG(FATAL) << "Nav cam must be in --camera_topics (unless using simulated data) as it is "
               << "needed for camera localization.\n";
  if (FLAGS_simulated_data && do_nav_cam)
    LOG(FATAL) << "The geometry mapper does not support nav_cam with simulated data as "
               << "its distortion is not modeled.\n";
  if (FLAGS_simulated_data && !do_haz_cam_image)
    LOG(FATAL) << "The haz_cam must be one of the camera types in simulation mode "
               << "as it is needed to read the simulated camera pose in order to "
               << "process the depth clouds.";

  // Parse the camera topics
  std::map<std::string, std::string> cam_topics;
  std::istringstream ifst(FLAGS_camera_topics);
  while (ifst >> str) {
    size_t count = cam_topics.size();
    if (count >= cam_types.size())
      LOG(FATAL) << "There must be a topic for each camera type.\n";
    cam_topics[cam_types[count]] = str;
    std::cout << "Camera topic for " << cam_types[count] << ": " << cam_topics[cam_types[count]]
              << "\n";
  }
  if (cam_types.size() != cam_topics.size())
    LOG(FATAL) << "There must be a topic for each camera type.\n";

  if (!FLAGS_simulated_data && !do_haz_cam_image) {
    // Even if it is not wanted to process the haz cam image, need to
    // have the haz cam topic to be able to process the cloud.
    cam_types.push_back("haz_cam");
    // The topic below won't be used, but haz to be in for the
    // bookkeeping to be correct.
    cam_topics["haz_cam"] = "/hw/depth_haz/extended/amplitude_int";
  }

  // Read simulated poses
  std::vector<dense_map::StampedPoseStorage> sim_cam_poses(cam_types.size());
  if (FLAGS_simulated_data) {
    for (size_t it = 0; it < cam_types.size(); it++) {
      std::string pose_topic = "/sim/" + cam_types[it] + "/pose";
      std::cout << "Pose topic for simulated camera: " << cam_types[it] << ": "
                << pose_topic << "\n";
      dense_map::readBagPoses(FLAGS_ros_bag, pose_topic, sim_cam_poses[it]);
    }
  }

  // Set up handles for reading data at given time stamp without
  // searching through the whole bag each time. Must use pointers
  // due to the rosbag API.
  std::map<std::string, boost::shared_ptr<dense_map::RosBagHandle>> bag_handles;
  for (size_t it = 0; it < cam_types.size(); it++) {
    bag_handles[cam_types[it]] = boost::shared_ptr<dense_map::RosBagHandle>
      (new dense_map::RosBagHandle(FLAGS_ros_bag, cam_topics[cam_types[it]]));
  }

  dense_map::RosBagHandle haz_cam_points_handle(FLAGS_ros_bag, FLAGS_haz_cam_points_topic);
  dense_map::RosBagHandle exif_handle(FLAGS_ros_bag, FLAGS_sci_cam_exif_topic);

  std::vector<double> nav_cam_bag_timestamps;
  if (!FLAGS_simulated_data)
    dense_map::readBagImageTimestamps(FLAGS_ros_bag, cam_topics["nav_cam"], nav_cam_bag_timestamps);

  std::map<double, std::vector<double>> sci_cam_exif;
  if (!FLAGS_simulated_data) dense_map::readExifFromBag(exif_handle.bag_msgs, sci_cam_exif);

  // How many columns to exclude from the left and the right ends of
  // the haz camera depth cloud (this is needed because that cloud has
  // some distortion).
  std::cout << "Cutting off " << FLAGS_depth_exclude_columns << " columns and "
            << FLAGS_depth_exclude_rows << " rows at the margins of the depth point clouds.\n";

  std::vector<camera::CameraParameters> cam_params;
  std::vector<Eigen::Affine3d>          nav_to_cam_trans;
  std::vector<double>                   nav_to_cam_timestamp_offset;
  Eigen::Affine3d nav_cam_to_body_trans;  // Will not be used
  Eigen::Affine3d                       haz_cam_depth_to_image_transform;
  if (!FLAGS_simulated_data) {
    dense_map::readConfigFile(  // Inputs
      cam_types,
      "nav_cam_transform",  // this is the nav cam to body transform
      "haz_cam_depth_to_image_transform",
      // Outputs
      cam_params, nav_to_cam_trans, nav_to_cam_timestamp_offset, nav_cam_to_body_trans,
      haz_cam_depth_to_image_transform);

    if (FLAGS_nav_cam_to_sci_cam_offset_override_value != "") {
      for (size_t it = 0; it < cam_types.size(); it++) {
        if (cam_types[it] == "sci_cam") {
          double new_val = atof(FLAGS_nav_cam_to_sci_cam_offset_override_value.c_str());
          std::cout << "Overriding the value " << nav_to_cam_timestamp_offset[it]
                    << " of nav_cam_to_sci_cam_timestamp_offset with: " << new_val << std::endl;
          nav_to_cam_timestamp_offset[it] = new_val;
        }
      }
    }
  } else {
    // No modeling of timestamp offset is used with simulated data. The
    // transforms between the cameras are not needed as we record each
    // camera pose.
    nav_to_cam_timestamp_offset = std::vector<double>(cam_types.size(), 0);
    for (size_t it = 0; it < cam_types.size(); it++)
      nav_to_cam_trans.push_back(Eigen::Affine3d::Identity());
    haz_cam_depth_to_image_transform = Eigen::Affine3d::Identity();  // no adjustment needed
  }

  boost::shared_ptr<sparse_mapping::SparseMap> sparse_map;
  if (!FLAGS_simulated_data) {
    std::cout << "Loading sparse map " << FLAGS_sparse_map << "\n";
    sparse_map = boost::shared_ptr<sparse_mapping::SparseMap>
      (new sparse_mapping::SparseMap(FLAGS_sparse_map));
    if (!FLAGS_use_brisk_map &&
        (sparse_map->GetDetectorName() != "SURF" || sparse_map->vocab_db_.binary_db != NULL)) {
      LOG(FATAL) << "A SURF map with no vocab db is expected, unless --use_brisk_map is specified.";
    }
    if (FLAGS_use_brisk_map &&
        (sparse_map->GetDetectorName() != "ORGBRISK" || sparse_map->vocab_db_.binary_db == NULL)) {
      LOG(FATAL) << "An ORGBRISK map with a vocab db is expected with --use_brisk_map.";
    }
  }

  // A very important sanity check. Not how we use gflags programatically
  // to access the value of --histogram_equalization as a string encoding a bool
  // ("true" or "false"). Here we ensure that the same flag for histogram
  // equalization is used in the map and for localization.
  std::string histogram_equalization;
  if (!FLAGS_simulated_data &&
      gflags::GetCommandLineOption("histogram_equalization", &histogram_equalization)) {
    if ((histogram_equalization == "true" && !sparse_map->GetHistogramEqualization()) ||
        (histogram_equalization == "false" && sparse_map->GetHistogramEqualization()))
      LOG(FATAL) << "The histogram equalization option in the sparse map and then one desired "
                 << "to use for localization are incompatible.";
  }

  // If desired to process only specific timestamps
  std::set<double> sci_cam_timestamps;
  if (FLAGS_sci_cam_timestamps != "") {
    std::ifstream ifs(FLAGS_sci_cam_timestamps.c_str());
    double val;
    while (ifs >> val) sci_cam_timestamps.insert(val);
  }

  // Create output directories
  dense_map::createDir(FLAGS_output_dir);
  std::map<std::string, std::string> cam_dirs;
  for (size_t it = 0; it < cam_types.size(); it++) {
    cam_dirs[cam_types[it]] = FLAGS_output_dir + "/distorted_" + cam_types[it];
    dense_map::createDir(cam_dirs[cam_types[it]]);
  }

  // The nav cam msgs may not exist for simulated data
  std::vector<rosbag::MessageInstance> empty_nav_cam_msgs;
  std::vector<rosbag::MessageInstance> * nav_cam_msgs_ptr;
  if (FLAGS_simulated_data)
    nav_cam_msgs_ptr = &empty_nav_cam_msgs;
  else
    nav_cam_msgs_ptr = &bag_handles["nav_cam"]->bag_msgs;

  // Save camera poses and other data for all desired cameras
  for (size_t it = 0; it < cam_types.size(); it++) {
    std::string cam_type = cam_types[it];

    if (cam_type == "nav_cam" && FLAGS_simulated_data) {
      std::cout << "The nav_cam is not supported with simulated data.\n";
      continue;
    }

    if (cam_type == "nav_cam" && !FLAGS_use_brisk_map && !FLAGS_simulated_data) {
      // Take a shortcut, since the sparse map is made of precisely of
      // the images we want to texture. Just save those images and
      // their poses from the map, avoiding localization.
      save_nav_cam_poses_and_images(sparse_map,
                                    *nav_cam_msgs_ptr,
                                    FLAGS_output_dir, cam_dirs["nav_cam"]);
      continue;
    }

    saveCameraPoses(FLAGS_simulated_data, cam_type, nav_to_cam_timestamp_offset[it],   // NOLINT
                    nav_to_cam_trans[it], haz_cam_depth_to_image_transform,            // NOLINT
                    FLAGS_depth_exclude_columns, FLAGS_depth_exclude_rows,             // NOLINT
                    FLAGS_foreshortening_delta, FLAGS_depth_hole_fill_diameter,        // NOLINT
                    FLAGS_reliability_weight_exponent, median_filter_params,           // NOLINT
                    bag_handles[cam_type]->bag_msgs,   // desired cam msgs             // NOLINT
                    *nav_cam_msgs_ptr,                 // nav msgs                     // NOLINT
                    haz_cam_points_handle.bag_msgs, nav_cam_bag_timestamps,            // NOLINT
                    sci_cam_exif, FLAGS_output_dir,                                    // NOLINT
                    cam_dirs[cam_type], FLAGS_start, FLAGS_duration,                   // NOLINT
                    FLAGS_sampling_spacing_seconds, FLAGS_dist_between_processed_cams, // NOLINT
                    sci_cam_timestamps, FLAGS_max_iso_times_exposure,                  // NOLINT
                    sparse_map, FLAGS_use_brisk_map,                                   // NOLINT
                    do_haz_cam_image, sim_cam_poses[it], FLAGS_save_debug_data);       // NOLINT

    if (FLAGS_simulated_data)
      saveSimCamIntrinsics(FLAGS_ros_bag, cam_type, cam_dirs[cam_type]);
  }

  return 0;
}
