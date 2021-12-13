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

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
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

// Read from the bag in intrinsics information for the simulated sci cam and save it to disk
void saveSciCamIntrinsics(std::string const& bag_file, std::string const& output_dir) {
  int image_width, image_height;
  double focal_length, optical_center_x, optical_center_y;

  std::vector<std::string> topics;
  std::string topic_sci_cam_sim_info = std::string("/") + TOPIC_SCI_CAM_SIM_INFO;
  topics.push_back(topic_sci_cam_sim_info);

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
    LOG(FATAL) << "Could not read sci cam intrinsics from: " << bag_file << " on topic: " << topic_sci_cam_sim_info;
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
  ofs << image_width << ' ' << image_height << ' ' << focal_length << ' ' << optical_center_x << ' ' << optical_center_y
      << "\n";
  ofs.close();
}

// Given a nav cam timestamp, find at most 4 images in the sparse map
// around that timestamp. Note that we use the counter sparse_id to
// travel forward in time through the sparse map upon repeated
// invocations of this function to narrow down the search.
bool findNearbySparseMapImages(double desired_nav_cam_time, std::vector<double> const& sparse_map_timestamps,
                               int& sparse_map_id,  // gets modified
                               std::vector<int>& nearby_cid) {
  int num_cid = sparse_map_timestamps.size();
  if (num_cid < 2) LOG(FATAL) << "Must have a map with at least two images.";

  // Wipe the output
  nearby_cid.clear();

  // If we overstep the bounds of the sparse map timestamps (when
  // using the SURF map with the map made up of the same images that
  // we want to texture) localization may become unreliable.
  bool within_bounds =
    (sparse_map_timestamps[0] <= desired_nav_cam_time && desired_nav_cam_time <= sparse_map_timestamps[num_cid - 1]);
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
// list of navcam image timestamps, find the nearest navcam images in
// time to the desired timestamp, localize from the sparse map their
// poses, then interpolate and transform to find the pose of the
// desired camera.
bool findInterpolatedPose(double desired_nav_cam_time, std::vector<double> const& nav_cam_bag_timestamps,
                          Eigen::MatrixXd const& desired_cam_to_nav_cam_transform,
                          std::vector<rosbag::MessageInstance> const& nav_cam_msgs,
                          boost::shared_ptr<sparse_mapping::SparseMap> sparse_map, bool use_brisk_map,
                          std::vector<double> const& sparse_map_timestamps, Eigen::MatrixXd& desired_cam_to_world_trans,
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
      if (!dense_map::lookupImage(beg_nav_cam_time, nav_cam_msgs, save_grayscale, beg_image, nav_cam_pos, found_time))
        return false;

      camera::CameraModel beg_localized_cam(Eigen::Vector3d(), Eigen::Matrix3d::Identity(),
                                            sparse_map->GetCameraParameters());
      if (!sparse_map->Localize(beg_image, &beg_localized_cam, NULL, NULL, nearby_cid_ptr)) return false;
      Eigen::Affine3d beg_trans = beg_localized_cam.GetTransform();

      // Note that below we use a throw-away value for nav_cam_pos.
      // That is because we would otherwise violate the assumption
      // that nav_cam_pos always goes forward in time. The position of
      // the end image now can be later in time than the position of
      // the beg image next time around.
      int end_nav_cam_pos = nav_cam_pos;
      cv::Mat end_image;
      if (!dense_map::lookupImage(end_nav_cam_time, nav_cam_msgs, save_grayscale, end_image, end_nav_cam_pos,
                                  found_time))
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

      desired_cam_to_world_trans = interp_trans.inverse().matrix() * desired_cam_to_nav_cam_transform;

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

  if (count != num_points) LOG(FATAL) << "Bookkeeping error. Did not add all the points to the depth map.";
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
          if (angle < foreshortening_delta || 180.0 - angle < foreshortening_delta || std::isnan(angle) ||
              std::isinf(angle)) {
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
void smooth_additions(cv::Mat const& origMat, cv::Mat& depthMat, cv::Mat& workMat, double sigma, Vec5d const& Zero,
                      int radius) {
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
void calc_weights(cv::Mat& depthMat, double exponent) {
  for (int row = 0; row < depthMat.rows; row++) {
#pragma omp parallel for
    for (int col = 0; col < depthMat.cols; col++) {
      if (zeroXyz(depthMat.at<Vec5d>(row, col))) continue;

      double drow = row - depthMat.rows / 2.0;
      double dcol = col - depthMat.cols / 2.0;
      double dist_sq = drow * drow + dcol * dcol;

      // Some kind of function decaying from center
      float weight = 1.0 / pow(1.0 + dist_sq, exponent/2.0);

      // Also, points that are further in z are given less weight
      double z = depthMat.at<Vec5d>(row, col)[2];
      weight *= 1.0 / (0.001 + z * z);

      // Make sure the weight does not get too small as voxblox
      // will read it as a float. Here making the weights
      // just a little bigger than what voxblox will accept.
      weight = std::max(weight, static_cast<float>(1.1e-6));

      depthMat.at<Vec5d>(row, col)[4] = weight;
    }
  }
}

void save_proc_depth_cloud(sensor_msgs::PointCloud2::ConstPtr pc_msg, cv::Mat const& haz_cam_intensity,
                           Eigen::Affine3d const& hazcam_depth_to_image_transform, int depth_exclude_columns,
                           int depth_exclude_rows, double foreshortening_delta, double depth_hole_fill_diameter,
                           double reliability_weight_exponent, std::vector<double> const& median_filter_params,
                           bool save_debug_data, const char* filename_buffer,
                           Eigen::MatrixXd const& desired_cam_to_world_trans) {
  // Sanity checks
  if (static_cast<int>(haz_cam_intensity.cols) != static_cast<int>(pc_msg->width) ||
      static_cast<int>(haz_cam_intensity.rows) != static_cast<int>(pc_msg->height))
    LOG(FATAL) << "Depth image dimensions does not agree with point cloud dimensions.";
  if (dense_map::matType(haz_cam_intensity) != "8UC1") LOG(FATAL) << "Extracted depth image should be of type: 8UC1.";

  // Extract the depth point cloud from the message
  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(*pc_msg, pc);
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
      double i = haz_cam_intensity.at<uchar>(row, col);

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
  int num_filter_passes = median_filter_params.size() / 2;
  for (int pass = 0; pass < num_filter_passes; pass++)
    median_filter(depthMat, workMat, Zero, median_filter_params[2 * pass + 0], median_filter_params[2 * pass + 1]);

  // Make a copy of the current depth image before filling holes
  cv::Mat origMat;
  depthMat.copyTo(origMat);

  // Fill holes using these radii
  int radius1 = depth_hole_fill_diameter / 2.0;
  // Use a smaller radius for the foreshortening constraint
  int radius2 = depth_hole_fill_diameter / 4.0;
  double sigma = 0.5;  // gaussian sigma used for hole-filling and smoothing
  if (depth_hole_fill_diameter > 0) hole_fill(depthMat, workMat, sigma, Zero, radius1, radius2, foreshortening_delta);

  // Smooth newly added points and also their immediate neighbors.
  // Use origMat for reference.
  int radius = depth_hole_fill_diameter / 4.0;
  if (depth_hole_fill_diameter > 0) smooth_additions(origMat, depthMat, workMat, sigma, Zero, radius);

  // This is the right place at which to add weights, before adding
  // extra points messes up with the number of rows in in depthMat.
  calc_weights(depthMat, reliability_weight_exponent);

  // Add extra points, but only if we are committed to manipulating the
  // depth cloud to start with
  if (depth_hole_fill_diameter > 0) add_extra_pts(depthMat, workMat, Zero);

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
      X = hazcam_depth_to_image_transform * X;
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
    // Find the image of distances in depthMat, for debugging.  Note
    // that we ignore the extra points we added as new rows depthMat in
    // add_extra_pts().

    // The depthMat may have extra rows, but we assume the number of columns
    // did not change.
    if (haz_cam_intensity.cols != depthMat.cols) LOG(FATAL) << "Incorrect number of columns in depthMat\n";

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
        X = hazcam_depth_to_image_transform * X;

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

#if 0

//  This is an instructive code to add to saveCameraPoses()
// to compare the sci cam pose as found via localizing
// the nav cam images vs localizing the sci cam image directly.
// The conclusion is that the sci cam calibration needs
// improvement, as the results are all over the place.
void localize_sci_cam_directly() {
  std::cout.precision(8);
  while (cam_type == "sci_cam" && !use_brisk_map) {
    // Find nav cam images around the nav cam pose to compute
    std::vector<int> nearby_cid;
    double desired_nav_cam_time = curr_time + desired_cam_to_nav_cam_offset;
    int local_sparse_map_id = 0;
    if (!findNearbySparseMapImages(desired_nav_cam_time,
                                   sparse_map_timestamps,
                                   local_sparse_map_id,  // gets modified
                                   nearby_cid)) {
      std::cout << "Fail to find nearby cid!" << std::endl;
      break;
    }
    std::cout << "Pose from using nav cam\n" << desired_cam_to_world_trans << std::endl;

    cv::Mat curr_sci_cam_image;
    bool curr_save_grayscale = true;
    double found_time = -1.0;
    if (!dense_map::lookupImage(curr_time, *desired_cam_msgs, curr_save_grayscale,
                                curr_sci_cam_image, desired_cam_pos, found_time)) {
      std::cout << "Failed to look up image!" << std::endl;
      break;
    } else {
      std::cout << "Success in finding sci cam image!" << std::endl;
    }

    snprintf(filename_buffer, sizeof(filename_buffer),
             "%s/%10.7f_grayscale.jpg", desired_cam_dir.c_str(), curr_time);
    std::cout << "Writing: " << filename_buffer << std::endl;
    cv::imwrite(filename_buffer, curr_sci_cam_image);

    double scale = 0.25;
    std::cout << "resizing with scale " << scale << std::endl;
    cv::Mat resized_image;
    cv::resize(curr_sci_cam_image, resized_image, cv::Size(), scale, scale, cv::INTER_AREA);

    std::cout << "Resized image dims: " << resized_image.cols << ' ' << resized_image.rows << "\n";

    camera::CameraModel curr_sci_cam(Eigen::Vector3d(), Eigen::Matrix3d::Identity(),
                                     g_sci_cam_params);
    if (!sparse_map->Localize(resized_image, &curr_sci_cam, NULL, NULL, &nearby_cid)) {
      std::cout << "Failed to localize!" << std::endl;
      break;
    } else {
      std::cout << "Success localizing!" << std::endl;
    }

    Eigen::Affine3d curr_trans = curr_sci_cam.GetTransform();
    Eigen::MatrixXd  T = (curr_trans.inverse()).matrix();
    std::cout << "sci cam mat\n" << T << std::endl;

    Eigen::Vector3d P(T(0, 3), T(1, 3), T(2, 3));
    std::cout << "Position " << P.transpose() << std::endl;

    double best_dist = 1e+100;
    double best_delta = -1.0;
    Eigen::MatrixXd best_s2n;

    for (double delta = -0.5; delta <= 0.5; delta += 0.05) {
      std::cout << "delta is " << delta << std::endl;

      int local_sparse_map_id = 0;
      int local_nav_cam_bag_id = 0;
      int local_nav_cam_pos = 0;
      Eigen::MatrixXd S;

      if (!findInterpolatedPose(curr_time + desired_cam_to_nav_cam_offset + delta,
                                nav_cam_bag_timestamps, desired_cam_to_nav_cam_transform,
                                nav_cam_msgs, sparse_map, use_brisk_map,
                                sparse_map_timestamps,
                                S,
                                local_sparse_map_id,
                                local_nav_cam_bag_id,
                                local_nav_cam_pos)) {
        std::cout << "Failed at local interp" << std::endl;
        continue;
      }
      std::cout << "Local trans for delta:\n" << S << std::endl;
      Eigen::Vector3d Q(S(0, 3), S(1, 3), S(2, 3));
      double local_dist = (Q-P).norm();
      std::cout << "delta position and norm "
                << delta << ' ' << Q.transpose()  << ' ' << local_dist << std::endl;

      Eigen::MatrixXd s2n = desired_cam_to_nav_cam_transform * (S.inverse()) * T;

      if (local_dist < best_dist) {
        best_dist = local_dist;
        best_delta = delta;
        best_s2n = s2n;
      }

      if (std::abs(delta) < 1e-10) {
        std::cout << "best s2n1\n" << s2n << std::endl;
        std::cout << "actual s2n\n" << desired_cam_to_nav_cam_transform << std::endl;
      }
    }

    std::cout << "best delta and dist " << best_delta << ' ' << best_dist << std::endl;
    std::cout << "best s2n2\n" << best_s2n << std::endl;
    break;
  }
}
#endif

  // Given a desired camera and a set of acquisition timestamps for it,
  // find the nav cam images nearest in time bracketing each timestamp,
  // find the poses of those images using localization, interpolate the
  // nav cam pose at the desired timestamp, and apply the transform
  // between the nav cam and the desired camera to find the pose of the
  // desired camera (by which it is meant the transform from the desired
  // camera to the world).

  // Also write the extracted point clouds and the desired cam (nav or sci) images.

  void saveCameraPoses(bool simulated_data, std::string const& cam_type,
                       double desired_cam_to_nav_cam_offset,
                       Eigen::MatrixXd const& desired_cam_to_nav_cam_transform,
                       Eigen::Affine3d& hazcam_depth_to_image_transform,
                       int depth_exclude_columns, int depth_exclude_rows,
                       double foreshortening_delta, double depth_hole_fill_diameter,
                       double reliability_weight_exponent,
                       std::vector<double> const& median_filter_params,
                       bool save_debug_data,
                       std::vector<rosbag::MessageInstance> const& nav_cam_msgs,
                       std::vector<rosbag::MessageInstance> const& sci_cam_msgs,
                       std::vector<rosbag::MessageInstance> const& haz_cam_points_msgs,
                       std::vector<rosbag::MessageInstance> const& haz_cam_intensity_msgs,
                       std::vector<double> const& nav_cam_bag_timestamps,
                       std::map<double, std::vector<double>> const& sci_cam_exif,
                       std::string const& output_dir, std::string const& desired_cam_dir,
                       double start, double duration,
                       double sampling_spacing_seconds,
                       double dist_between_processed_cams, std::set<double> const& sci_cam_timestamps,
                       double max_iso_times_exposure,
                       boost::shared_ptr<sparse_mapping::SparseMap> sparse_map, bool use_brisk_map,
                       dense_map::StampedPoseStorage const& sim_desired_cam_poses,
                       std::string const& external_mesh) {
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

    // Open a handle for writing the index of files being written to disk
    std::string index_file = output_dir + "/" + cam_type + "_index.txt";
    std::ofstream ofs(index_file.c_str());

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

    // Decide on which messages to process
    std::vector<rosbag::MessageInstance> const* desired_cam_msgs = NULL;
    if (cam_type == "haz_cam") {
      desired_cam_msgs = &haz_cam_points_msgs;  // points topic
    } else if (cam_type == "sci_cam") {
      desired_cam_msgs = &sci_cam_msgs;  // image topic
    } else if (cam_type == "nav_cam") {
      desired_cam_msgs = &nav_cam_msgs;  // image topic
    } else {
      LOG(FATAL) << "Unknown camera type: " << cam_type;
    }

    if (external_mesh == "") {
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
    } else {
      // Use an external mesh, so ignore the depth clouds (may even be absent)
      beg_time = min_map_timestamp;
    }

    for (auto& m : *desired_cam_msgs) {
      sensor_msgs::PointCloud2::ConstPtr pc_msg;
      double curr_time = -1.0;

      if (cam_type == "haz_cam") {
        // haz cam
        pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (!pc_msg) continue;
        curr_time = pc_msg->header.stamp.toSec();
      } else {
        // sci cam or nav cam
        // Check for image
        sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
        if (image_msg) {
          curr_time = image_msg->header.stamp.toSec();
        } else {
          // Check for compressed image
          sensor_msgs::CompressedImage::ConstPtr comp_image_msg = m.instantiate<sensor_msgs::CompressedImage>();
          if (comp_image_msg) {
            curr_time = comp_image_msg->header.stamp.toSec();
          } else {
            // Not an image or compressed image topic
            continue;
          }
        }
      }

      bool custom_timestamps = (cam_type == "sci_cam" && !sci_cam_timestamps.empty());
      if (!simulated_data && !use_brisk_map && !custom_timestamps) {
        if (curr_time < min_map_timestamp) continue;  // Do not process data before the sparse map starts
        if (curr_time > max_map_timestamp) break;     // Do not process data after the sparse map ends
      }

      if (curr_time - beg_time < start) continue;  // did not yet reach the desired starting time

      if (duration > 0 && curr_time - beg_time > start + duration) break;  // past the desired ending time

      // The pose we will solve for
      bool lookup_success = false;
      Eigen::MatrixXd desired_cam_to_world_trans;

      // Let some time elapse between fusing clouds
      if (std::abs(curr_time - prev_time) < sampling_spacing_seconds && !custom_timestamps) {
        continue;
      }

      // The case of custom timestamps
      if (custom_timestamps && sci_cam_timestamps.find(curr_time) == sci_cam_timestamps.end()) continue;

      if (!simulated_data) {
        // Do localization based on nav cam, transform the pose to desired cam coordinates, etc
        // Note that we convert from the desired time to the nav cam time for this operation
        if (!findInterpolatedPose(curr_time + desired_cam_to_nav_cam_offset, nav_cam_bag_timestamps,
                                  desired_cam_to_nav_cam_transform, nav_cam_msgs, sparse_map, use_brisk_map,
                                  sparse_map_timestamps, desired_cam_to_world_trans, sparse_map_id, nav_cam_bag_id,
                                  nav_cam_pos)) {
          std::cout.precision(17);
          std::cout << "Localization failed at time " << curr_time << std::endl;
          std::cout << "if too many such failures, consider modifying the value of "
                    << "--localization_options. See the documentation for more info." << std::endl;
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
      if (!std::isnan(prev_cam_ctr[0]) && !custom_timestamps && curr_dist_bw_cams < dist_between_processed_cams) {
        std::cout << "Distance to previously processed camera is " << curr_dist_bw_cams << ", skipping it."
                  << std::endl;
        continue;
      }

      char filename_buffer[1000];
      std::string suffix = cam_type + "_to_world";

      if (cam_type == "haz_cam") {
        cv::Mat haz_cam_intensity;
        bool save_grayscale = true;

        // Normally try to look up the image time right after the current cloud time
        double desired_image_time = curr_time;
        double found_time = -1.0;
        if (!dense_map::lookupImage(desired_image_time, haz_cam_intensity_msgs, save_grayscale, haz_cam_intensity,
                                    desired_cam_pos, found_time))
          continue;

        // Save the transform (only if the above lookup was successful)
        snprintf(filename_buffer, sizeof(filename_buffer), "%s/%10.7f_%s.txt", output_dir.c_str(), curr_time,
                 suffix.c_str());
        dense_map::writeMatrix(desired_cam_to_world_trans, filename_buffer);

        // Save the transform name in the index file
        ofs << filename_buffer << "\n";

        // Save the cloud
        snprintf(filename_buffer, sizeof(filename_buffer), "%s/%10.7f.pcd", output_dir.c_str(), curr_time);

        // Append the intensity to the point cloud and save it
        save_proc_depth_cloud(pc_msg, haz_cam_intensity, hazcam_depth_to_image_transform, depth_exclude_columns,
                              depth_exclude_rows, foreshortening_delta, depth_hole_fill_diameter,
                              reliability_weight_exponent, median_filter_params, save_debug_data, filename_buffer,
                              desired_cam_to_world_trans);

        // Save the name of the point cloud to the index
        ofs << filename_buffer << "\n";

      } else {
        // Save the nav or sci cam image at the given timestamp.
        cv::Mat desired_cam_image;
        bool save_grayscale = false;
        double found_time = -1.0;
        if (!dense_map::lookupImage(curr_time, *desired_cam_msgs, save_grayscale, desired_cam_image, desired_cam_pos,
                                    found_time))
          continue;

        // Save the transform (only if the above lookup was successful)
        snprintf(filename_buffer, sizeof(filename_buffer), "%s/%10.7f_%s.txt", output_dir.c_str(), curr_time,
                 suffix.c_str());
        dense_map::writeMatrix(desired_cam_to_world_trans, filename_buffer);

        // This is very useful for debugging things with ASP.

        // A debug utility for saving a camera in format ASP understands
        // if (cam_type == "sci_cam")
        //   dense_map::save_tsai_camera(desired_cam_to_world_trans, output_dir, curr_time, suffix);

        // Apply an optional scale. Use a pointer to not copy the data more than
        // one has to.
        cv::Mat* img_ptr = &desired_cam_image;
        cv::Mat scaled_image;
        if (!simulated_data && cam_type == "sci_cam") {
          auto exif_it = sci_cam_exif.find(curr_time);
          if (exif_it != sci_cam_exif.end()) {
            std::vector<double> const& exif = exif_it->second;
            double iso = exif[dense_map::ISO];
            double exposure = exif[dense_map::EXPOSURE_TIME];
            dense_map::exposureCorrection(max_iso_times_exposure, iso, exposure, *img_ptr, scaled_image);
            img_ptr = &scaled_image;
          }
        }

        snprintf(filename_buffer, sizeof(filename_buffer), "%s/%10.7f.jpg", desired_cam_dir.c_str(), curr_time);
        std::cout << "Writing: " << filename_buffer << std::endl;
        cv::imwrite(filename_buffer, *img_ptr);

        // Save the name of the image in the index
        ofs << filename_buffer << "\n";
      }

      // Update the previous camera center
      prev_cam_ctr = curr_cam_ctr;
    }

    std::cout << "Wrote: " << index_file << std::endl;
  }

  void save_navcam_poses_and_images(boost::shared_ptr<sparse_mapping::SparseMap> sparse_map,
                                    std::vector<rosbag::MessageInstance> const& nav_cam_msgs,
                                    std::string const& output_dir, std::string const& nav_cam_dir) {
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
      if (cam_file == img_file) LOG(FATAL) << "Failed to replace the image extension in: " << img_file;

      dense_map::writeMatrix(sparse_map->cid_to_cam_t_global_[cid].inverse().matrix(), cam_file);

      cv::Mat nav_cam_image;
      bool save_grayscale = false;
      double found_time = -1.0;
      if (!dense_map::lookupImage(timestamp, nav_cam_msgs, save_grayscale, nav_cam_image, nav_cam_pos, found_time))
        continue;

      snprintf(filename_buffer, sizeof(filename_buffer), "%s/%10.7f.jpg", nav_cam_dir.c_str(), timestamp);
      std::cout << "Writing: " << filename_buffer << std::endl;
      cv::imwrite(filename_buffer, nav_cam_image);

      ofs << filename_buffer << "\n";
    }
  }

  DEFINE_string(ros_bag, "", "A ROS bag with recorded nav_cam, haz_cam, and sci_cam data.");
  DEFINE_string(sparse_map, "", "A registered sparse map made with some of the ROS bag data.");
  DEFINE_string(output_dir, "", "The full path to a directory where to write the processed data.");
  DEFINE_string(nav_cam_topic, "/hw/cam_nav", "The nav cam topic in the bag file.");
  DEFINE_string(haz_cam_points_topic, "/hw/depth_haz/points", "The depth point cloud topic in the bag file.");
  DEFINE_string(haz_cam_intensity_topic, "/hw/depth_haz/extended/amplitude_int",
                "The depth camera intensity topic in the bag file.");
  DEFINE_string(sci_cam_topic, "/hw/cam_sci", "The sci cam topic in the bag file.");
  DEFINE_string(sci_cam_exif_topic, "/hw/sci_cam_exif", "The sci cam exif metadata topic the output bag.");
  DEFINE_string(camera_type, "all",
                "Specify which cameras to process. By default, process all. Options: "
                "nav_cam, sci_cam.");
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
  DEFINE_string(external_mesh, "", "Use this mesh to texture the images, rather than creating one from depth data.");
  DEFINE_string(scicam_to_hazcam_timestamp_offset_override_value, "",
                "Override the value of scicam_to_hazcam_timestamp_offset from the robot config "
                "file with this value.");
  DEFINE_bool(save_debug_data, false, "Save many intermediate datasets for debugging.");

  int main(int argc, char** argv) {
    ff_common::InitFreeFlyerApplication(&argc, &argv);
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    if (FLAGS_ros_bag.empty()) LOG(FATAL) << "The bag file was not specified.";
    if (FLAGS_output_dir.empty()) LOG(FATAL) << "The output directory was not specified.";
    if (FLAGS_sparse_map.empty() && !FLAGS_simulated_data) LOG(FATAL) << "The sparse map was not specified.";

    if (!boost::filesystem::exists(FLAGS_ros_bag)) LOG(FATAL) << "Bag does not exist: " << FLAGS_ros_bag;

    if (!boost::filesystem::exists(FLAGS_sparse_map) && !FLAGS_simulated_data)
      LOG(FATAL) << "Sparse map does not exist: " << FLAGS_sparse_map;

    // Parse the median filter params
    double val;
    std::vector<double> median_filter_params;
    std::istringstream ifs(FLAGS_median_filters);
    while (ifs >> val) median_filter_params.push_back(val);

    if (median_filter_params.size() % 2 != 0)
      LOG(FATAL) << "There must exist an even number of median filter parameters, "
                 << "being pairs of window sizes and distances.";

    // Need high precision to print timestamps
    std::cout.precision(17);

    // Read simulated poses
    dense_map::StampedPoseStorage sim_sci_cam_poses, sim_haz_cam_poses;
    if (FLAGS_simulated_data) {
      std::string haz_cam_pose_topic = std::string("/") + TOPIC_HAZ_CAM_SIM_POSE;
      std::string sci_cam_pose_topic = std::string("/") + TOPIC_SCI_CAM_SIM_POSE;
      std::cout << "haz cam pose topic " << haz_cam_pose_topic << std::endl;
      std::cout << "sci cam pose topic " << sci_cam_pose_topic << std::endl;
      dense_map::readBagPoses(FLAGS_ros_bag, haz_cam_pose_topic, sim_haz_cam_poses);
      dense_map::readBagPoses(FLAGS_ros_bag, sci_cam_pose_topic, sim_sci_cam_poses);
    }

    // Set up handles for reading data at given time stamp without
    // searching through the whole bag each time.
    dense_map::RosBagHandle nav_cam_handle(FLAGS_ros_bag, FLAGS_nav_cam_topic);
    dense_map::RosBagHandle sci_cam_handle(FLAGS_ros_bag, FLAGS_sci_cam_topic);
    dense_map::RosBagHandle haz_cam_points_handle(FLAGS_ros_bag, FLAGS_haz_cam_points_topic);
    dense_map::RosBagHandle haz_cam_intensity_handle(FLAGS_ros_bag, FLAGS_haz_cam_intensity_topic);
    dense_map::RosBagHandle exif_handle(FLAGS_ros_bag, FLAGS_sci_cam_exif_topic);

    std::vector<double> nav_cam_bag_timestamps;
    if (!FLAGS_simulated_data)
      dense_map::readBagImageTimestamps(FLAGS_ros_bag, FLAGS_nav_cam_topic, nav_cam_bag_timestamps);

    std::map<double, std::vector<double>> sci_cam_exif;
    if (!FLAGS_simulated_data) dense_map::readExifFromBag(exif_handle.bag_msgs, sci_cam_exif);

    // How many columns to exclude from the left and the right ends of
    // the haz camera depth cloud (this is needed because that cloud has
    // some distortion).
    std::cout << "Cutting off " << FLAGS_depth_exclude_columns << " columns and " << FLAGS_depth_exclude_rows
              << " rows at the margins of the depth point clouds.\n";

    double navcam_to_hazcam_timestamp_offset = 0.0, scicam_to_hazcam_timestamp_offset = 0.0;
    Eigen::MatrixXd hazcam_to_navcam_trans = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd scicam_to_hazcam_trans = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd scicam_to_navcam_trans = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd navcam_to_navcam_trans = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd navcam_to_body_trans = Eigen::MatrixXd::Identity(4, 4);
    Eigen::Affine3d hazcam_depth_to_image_transform;
    hazcam_depth_to_image_transform.setIdentity();  // default value
    camera::CameraParameters nav_cam_params(Eigen::Vector2i(0, 0), Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 0));
    camera::CameraParameters haz_cam_params(Eigen::Vector2i(0, 0), Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 0));
    camera::CameraParameters sci_cam_params(Eigen::Vector2i(0, 0), Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 0));
    if (!FLAGS_simulated_data) {
      // Read a bunch of transforms from the robot calibration file.
      // The empirically determined offset between the measured timestamps
      // of the cameras, in seconds. There are two reasons for this time offset:
      // (1) The nav cam and sci cam are acquired on different processors.
      // (2) The actual moment the camera image is recorded is not the same
      // moment that image finished processing and the timestamp is set.
      // This delay is worse for the sci cam which takes longer to acquire.
      // TODO(oalexan1): camera_calibrator must actually solve for them.
      dense_map::readConfigFile("navcam_to_hazcam_timestamp_offset", "scicam_to_hazcam_timestamp_offset",
                                "hazcam_to_navcam_transform", "scicam_to_hazcam_transform", "nav_cam_transform",
                                "hazcam_depth_to_image_transform", navcam_to_hazcam_timestamp_offset,
                                scicam_to_hazcam_timestamp_offset, hazcam_to_navcam_trans, scicam_to_hazcam_trans,
                                navcam_to_body_trans, hazcam_depth_to_image_transform, nav_cam_params, haz_cam_params,
                                sci_cam_params);

      if (FLAGS_scicam_to_hazcam_timestamp_offset_override_value != "") {
        double new_val = atof(FLAGS_scicam_to_hazcam_timestamp_offset_override_value.c_str());
        std::cout << "Overriding the value " << scicam_to_hazcam_timestamp_offset
                  << " of scicam_to_hazcam_timestamp_offset with: " << new_val << std::endl;
        scicam_to_hazcam_timestamp_offset = new_val;
      }

      scicam_to_navcam_trans = hazcam_to_navcam_trans * scicam_to_hazcam_trans;
      std::cout << "hazcam_to_navcam_trans\n" << hazcam_to_navcam_trans << std::endl;
      std::cout << "scicam_to_hazcam_trans\n" << scicam_to_hazcam_trans << std::endl;
      std::cout << "scicam_to_navcam_trans\n" << scicam_to_navcam_trans << std::endl;
      std::cout << "navcam_to_hazcam_timestamp_offset: " << navcam_to_hazcam_timestamp_offset << "\n";
      std::cout << "scicam_to_hazcam_timestamp_offset: " << scicam_to_hazcam_timestamp_offset << "\n";
      std::cout << "hazcam_depth_to_image_transform\n" << hazcam_depth_to_image_transform.matrix() << "\n";
    }

    boost::shared_ptr<sparse_mapping::SparseMap> sparse_map;
    if (!FLAGS_simulated_data) {
      std::cout << "Loading sparse map " << FLAGS_sparse_map << "\n";
      sparse_map = boost::shared_ptr<sparse_mapping::SparseMap>(new sparse_mapping::SparseMap(FLAGS_sparse_map));
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
    if (gflags::GetCommandLineOption("histogram_equalization", &histogram_equalization)) {
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

    dense_map::createDir(FLAGS_output_dir);

    // Store here the sci cam and nav cam images whose poses we save
    std::string sci_cam_dir = FLAGS_output_dir + "/distorted_sci_cam";
    std::string nav_cam_dir = FLAGS_output_dir + "/distorted_nav_cam";
    dense_map::createDir(sci_cam_dir);
    dense_map::createDir(nav_cam_dir);

    // Save haz cam poses and haz cam clouds
    if (FLAGS_external_mesh == "") {
      std::string cam_type = "haz_cam";
      // haz to nav offset
      double desired_cam_to_nav_cam_offset = -navcam_to_hazcam_timestamp_offset;
      saveCameraPoses(FLAGS_simulated_data, cam_type, desired_cam_to_nav_cam_offset, hazcam_to_navcam_trans,
                      hazcam_depth_to_image_transform, FLAGS_depth_exclude_columns, FLAGS_depth_exclude_rows,
                      FLAGS_foreshortening_delta, FLAGS_depth_hole_fill_diameter,
                      FLAGS_reliability_weight_exponent,
                      median_filter_params,
                      FLAGS_save_debug_data, nav_cam_handle.bag_msgs, sci_cam_handle.bag_msgs,
                      haz_cam_points_handle.bag_msgs, haz_cam_intensity_handle.bag_msgs, nav_cam_bag_timestamps,
                      sci_cam_exif, FLAGS_output_dir, "",  // not needed
                      FLAGS_start, FLAGS_duration, FLAGS_sampling_spacing_seconds, FLAGS_dist_between_processed_cams,
                      sci_cam_timestamps, FLAGS_max_iso_times_exposure, sparse_map, FLAGS_use_brisk_map,
                      sim_haz_cam_poses, FLAGS_external_mesh);
    }

    // Save sci cam poses and sci cam images
    if (FLAGS_camera_type == "sci_cam" || FLAGS_camera_type == "all") {
      std::string cam_type = "sci_cam";
      // sci to nav offset
      double desired_cam_to_nav_cam_offset = scicam_to_hazcam_timestamp_offset - navcam_to_hazcam_timestamp_offset;
      saveCameraPoses(FLAGS_simulated_data, cam_type, desired_cam_to_nav_cam_offset, scicam_to_navcam_trans,
                      hazcam_depth_to_image_transform, FLAGS_depth_exclude_columns, FLAGS_depth_exclude_rows,
                      FLAGS_foreshortening_delta, FLAGS_depth_hole_fill_diameter,
                      FLAGS_reliability_weight_exponent,
                      median_filter_params,
                      FLAGS_save_debug_data, nav_cam_handle.bag_msgs, sci_cam_handle.bag_msgs,
                      haz_cam_points_handle.bag_msgs, haz_cam_intensity_handle.bag_msgs, nav_cam_bag_timestamps,
                      sci_cam_exif, FLAGS_output_dir, sci_cam_dir, FLAGS_start, FLAGS_duration,
                      FLAGS_sampling_spacing_seconds, FLAGS_dist_between_processed_cams, sci_cam_timestamps,
                      FLAGS_max_iso_times_exposure, sparse_map, FLAGS_use_brisk_map, sim_sci_cam_poses,
                      FLAGS_external_mesh);

      // With simulated data we won't perform an additional undistortion
      // step which would save the undistorted intrinsics. Hence need to
      // do that here.
      if (FLAGS_simulated_data) saveSciCamIntrinsics(FLAGS_ros_bag, sci_cam_dir);
    }
    // Save nav cam poses and nav cam images. This does not work for simulated data.
    if ((FLAGS_camera_type == "nav_cam" || FLAGS_camera_type == "all") && !FLAGS_simulated_data) {
      if (FLAGS_use_brisk_map) {
        // Do the same logic as for the sci cam, localize each nav cam image against the map, etc.
        std::string cam_type = "nav_cam";
        double desired_cam_to_nav_cam_offset = 0.0;       // no offset from the nav cam to itself
        dense_map::StampedPoseStorage sim_nav_cam_poses;  // won't be used
        saveCameraPoses(FLAGS_simulated_data, cam_type, desired_cam_to_nav_cam_offset, navcam_to_navcam_trans,
                        hazcam_depth_to_image_transform, FLAGS_depth_exclude_columns, FLAGS_depth_exclude_rows,
                        FLAGS_foreshortening_delta, FLAGS_depth_hole_fill_diameter,
                        FLAGS_reliability_weight_exponent,
                        median_filter_params,
                        FLAGS_save_debug_data, nav_cam_handle.bag_msgs, sci_cam_handle.bag_msgs,
                        haz_cam_points_handle.bag_msgs, haz_cam_intensity_handle.bag_msgs, nav_cam_bag_timestamps,
                        sci_cam_exif, FLAGS_output_dir, nav_cam_dir, FLAGS_start, FLAGS_duration,
                        FLAGS_sampling_spacing_seconds, FLAGS_dist_between_processed_cams, sci_cam_timestamps,
                        FLAGS_max_iso_times_exposure, sparse_map, FLAGS_use_brisk_map, sim_nav_cam_poses,
                        FLAGS_external_mesh);
      } else {
        // Take a shortcut, since the sparse map is made of precisely of the images we
        // want to texture, just save those images and their poses from the map, avoiding
        // localization
        save_navcam_poses_and_images(sparse_map, nav_cam_handle.bag_msgs, FLAGS_output_dir, nav_cam_dir);
      }
    }

    return 0;
  }
