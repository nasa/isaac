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

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <interest_point.h>           // from the isaac repo
#include <camera_image.h>             // from the isaac repo
#include <interest_point/matching.h>  // from the astrobee repo

// Get rid of warnings beyond our control
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

// SIFT is doing so much better than SURF for haz cam images.
DEFINE_string(refiner_feature_detector, "SIFT", "The feature detector to use. SIFT or SURF.");
DEFINE_int32(sift_nFeatures, 10000, "Number of SIFT features");
DEFINE_int32(sift_nOctaveLayers, 3, "Number of SIFT octave layers");
DEFINE_double(sift_contrastThreshold, 0.02,
              "SIFT contrast threshold");  // decrease for more ip
DEFINE_double(sift_edgeThreshold, 10, "SIFT edge threshold");
DEFINE_double(sift_sigma, 1.6, "SIFT sigma");

namespace dense_map {

void detectFeatures(const cv::Mat& image, bool verbose,
                    // Outputs
                    cv::Mat* descriptors, Eigen::Matrix2Xd* keypoints) {
  bool histogram_equalization = false;

  // If using histogram equalization, need an extra image to store it
  cv::Mat* image_ptr = const_cast<cv::Mat*>(&image);
  cv::Mat hist_image;
  if (histogram_equalization) {
    cv::equalizeHist(image, hist_image);
    image_ptr = &hist_image;
  }

  std::vector<cv::KeyPoint> storage;

  if (FLAGS_refiner_feature_detector == "SIFT") {
    cv::Ptr<cv::xfeatures2d::SIFT> sift =
      cv::xfeatures2d::SIFT::create(FLAGS_sift_nFeatures, FLAGS_sift_nOctaveLayers, FLAGS_sift_contrastThreshold,
                                    FLAGS_sift_edgeThreshold, FLAGS_sift_sigma);
    sift->detect(image, storage);
    sift->compute(image, storage, *descriptors);

  } else if (FLAGS_refiner_feature_detector == "SURF") {
    interest_point::FeatureDetector detector("SURF");
    detector.Detect(*image_ptr, &storage, descriptors);

    // Undo the shift in the detector
    for (cv::KeyPoint& key : storage) {
      key.pt.x += image.cols / 2.0;
      key.pt.y += image.rows / 2.0;
    }

  } else {
    LOG(FATAL) << "Unknown feature detector: " << FLAGS_refiner_feature_detector;
  }

  if (verbose) std::cout << "Features detected " << storage.size() << std::endl;

  // Copy to data structures expected by subsequent code
  keypoints->resize(2, storage.size());
  Eigen::Vector2d output;
  for (size_t j = 0; j < storage.size(); j++) {
    keypoints->col(j) = Eigen::Vector2d(storage[j].pt.x, storage[j].pt.y);
  }
}

// This really likes haz cam first and nav cam second
void matchFeatures(std::mutex* match_mutex, int left_image_index, int right_image_index,
                   cv::Mat const& left_descriptors, cv::Mat const& right_descriptors,
                   Eigen::Matrix2Xd const& left_keypoints,
                   Eigen::Matrix2Xd const& right_keypoints, bool verbose,
                   // output
                   MATCH_PAIR* matches) {
  std::vector<cv::DMatch> cv_matches;
  interest_point::FindMatches(left_descriptors, right_descriptors, &cv_matches);

  std::vector<cv::Point2f> left_vec;
  std::vector<cv::Point2f> right_vec;
  for (size_t j = 0; j < cv_matches.size(); j++) {
    int left_ip_index = cv_matches.at(j).queryIdx;
    int right_ip_index = cv_matches.at(j).trainIdx;

    // Get the keypoints from the good matches
    left_vec.push_back(cv::Point2f(left_keypoints.col(left_ip_index)[0], left_keypoints.col(left_ip_index)[1]));
    right_vec.push_back(cv::Point2f(right_keypoints.col(right_ip_index)[0], right_keypoints.col(right_ip_index)[1]));
  }

  if (left_vec.empty()) return;

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
  for (size_t j = 0; j < cv_matches.size(); j++) {
    int left_ip_index = cv_matches.at(j).queryIdx;
    int right_ip_index = cv_matches.at(j).trainIdx;

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

void writeIpRecord(std::ofstream& f, InterestPoint const& p) {
  f.write(reinterpret_cast<const char*>(&(p.x)), sizeof(p.x));
  f.write(reinterpret_cast<const char*>(&(p.y)), sizeof(p.y));
  f.write(reinterpret_cast<const char*>(&(p.ix)), sizeof(p.ix));
  f.write(reinterpret_cast<const char*>(&(p.iy)), sizeof(p.iy));
  f.write(reinterpret_cast<const char*>(&(p.orientation)), sizeof(p.orientation));
  f.write(reinterpret_cast<const char*>(&(p.scale)), sizeof(p.scale));
  f.write(reinterpret_cast<const char*>(&(p.interest)), sizeof(p.interest));
  f.write(reinterpret_cast<const char*>(&(p.polarity)), sizeof(p.polarity));
  f.write(reinterpret_cast<const char*>(&(p.octave)), sizeof(p.octave));
  f.write(reinterpret_cast<const char*>(&(p.scale_lvl)), sizeof(p.scale_lvl));
  uint64_t size = p.size();
  f.write(reinterpret_cast<const char*>((&size)), sizeof(uint64));
  for (size_t i = 0; i < p.descriptor.size(); ++i)
    f.write(reinterpret_cast<const char*>(&(p.descriptor[i])), sizeof(p.descriptor[i]));
}

// Write matches to disk
void writeMatchFile(std::string match_file, std::vector<InterestPoint> const& ip1,
                    std::vector<InterestPoint> const& ip2) {
  std::ofstream f;
  f.open(match_file.c_str(), std::ios::binary | std::ios::out);
  std::vector<InterestPoint>::const_iterator iter1 = ip1.begin();
  std::vector<InterestPoint>::const_iterator iter2 = ip2.begin();
  uint64 size1 = ip1.size();
  uint64 size2 = ip2.size();
  f.write(reinterpret_cast<const char*>(&size1), sizeof(uint64));
  f.write(reinterpret_cast<const char*>(&size2), sizeof(uint64));
  for (; iter1 != ip1.end(); ++iter1) writeIpRecord(f, *iter1);
  for (; iter2 != ip2.end(); ++iter2) writeIpRecord(f, *iter2);
  f.close();
}

void saveImagesAndMatches(std::string const& left_prefix, std::string const& right_prefix,
                          std::pair<int, int> const& index_pair, MATCH_PAIR const& match_pair,
                          std::vector<cv::Mat> const& images) {
  // Add 10000 to have them be listed nicely
  std::ostringstream oss_left;
  oss_left << left_prefix << "_image" << index_pair.first + 10000 << ".jpg";
  std::string left_image_file = oss_left.str();
  std::cout << "Writing: " << left_image_file << std::endl;
  cv::imwrite(left_image_file, images[index_pair.first]);

  std::ostringstream oss_right;
  oss_right << right_prefix << "_image" << index_pair.second + 10000 << ".jpg";
  std::string right_image_file = oss_right.str();
  std::cout << "Writing: " << right_image_file << std::endl;
  cv::imwrite(right_image_file, images[index_pair.second]);

  std::string left_stem = boost::filesystem::path(left_image_file).stem().string();
  std::string right_stem = boost::filesystem::path(right_image_file).stem().string();

  std::string match_file = left_stem + "__" + right_stem + ".match";
  std::cout << "Writing: " << left_image_file << ' ' << right_image_file << ' ' << match_file << std::endl;
  writeMatchFile(match_file, match_pair.first, match_pair.second);
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

void detectMatchFeatures(  // Inputs
                         std::vector<dense_map::cameraImage> const& cams,
                         std::vector<std::string> const& cam_names,
                         std::vector<camera::CameraParameters> const& cam_params,
                         std::vector<Eigen::Affine3d> const& world_to_cam, int num_overlaps,
                         int initial_max_reprojection_error, int num_match_threads,
                         bool verbose,
                         // Outputs
                         std::vector<std::vector<std::pair<float, float>>>& keypoint_vec,
                         std::vector<std::map<int, int>>& pid_to_cid_fid,
                         std::vector<std::string> & image_files) {
  // Wipe the outputs
  keypoint_vec.clear();
  pid_to_cid_fid.clear();
  image_files.clear();

  if (verbose) {
    int count = 10000;
    for (size_t it = 0; it < cams.size(); it++) {
      std::ostringstream oss;
      oss << count << "_" << cam_names[cams[it].camera_type] << ".jpg";
      std::string name = oss.str();
      std::cout << "Writing: " << name << std::endl;
      cv::imwrite(name, cams[it].image);
      count++;
      image_files.push_back(name);
    }
  }

  // Detect features using multiple threads. Too many threads may result
  // in high memory usage.
  std::ostringstream oss;
  oss << num_match_threads;
  std::string num_threads = oss.str();
  google::SetCommandLineOption("num_threads", num_threads.c_str());
  if (!gflags::GetCommandLineOption("num_threads", &num_threads))
    LOG(FATAL) << "Failed to get the value of --num_threads in Astrobee software.\n";
  std::cout << "Using " << num_threads << " threads for feature detection/matching." << std::endl;

  std::cout << "Detecting features." << std::endl;

  std::vector<cv::Mat> cid_to_descriptor_map;
  std::vector<Eigen::Matrix2Xd> cid_to_keypoint_map;
  cid_to_descriptor_map.resize(cams.size());
  cid_to_keypoint_map.resize(cams.size());
  {
    // Make the thread pool go out of scope when not needed to not use up memory
    ff_common::ThreadPool thread_pool;
    for (size_t it = 0; it < cams.size(); it++) {
      thread_pool.AddTask
        (&dense_map::detectFeatures,    // multi-thread  // NOLINT
         // dense_map::detectFeatures(  // single-thread // NOLINT
         cams[it].image, verbose, &cid_to_descriptor_map[it], &cid_to_keypoint_map[it]);
    }
    thread_pool.Join();
  }

  MATCH_MAP matches;

  std::vector<std::pair<int, int> > image_pairs;
  for (size_t it1 = 0; it1 < cams.size(); it1++) {
    for (size_t it2 = it1 + 1; it2 < std::min(cams.size(), it1 + num_overlaps + 1); it2++) {
      image_pairs.push_back(std::make_pair(it1, it2));
    }
  }

  {
    std::cout << "Matching features." << std::endl;
    ff_common::ThreadPool thread_pool;
    std::mutex match_mutex;
    for (size_t pair_it = 0; pair_it < image_pairs.size(); pair_it++) {
      auto pair = image_pairs[pair_it];
      int left_image_it = pair.first, right_image_it = pair.second;
      thread_pool.AddTask
        (&dense_map::matchFeaturesWithCams,   // multi-threaded  // NOLINT
         // dense_map::matchFeaturesWithCams( // single-threaded // NOLINT
         &match_mutex, left_image_it, right_image_it, cam_params[cams[left_image_it].camera_type],
         cam_params[cams[right_image_it].camera_type], world_to_cam[left_image_it],
         world_to_cam[right_image_it], initial_max_reprojection_error,
         cid_to_descriptor_map[left_image_it], cid_to_descriptor_map[right_image_it],
         cid_to_keypoint_map[left_image_it], cid_to_keypoint_map[right_image_it], verbose,
         &matches[pair]);
    }
    thread_pool.Join();
  }
  cid_to_descriptor_map = std::vector<cv::Mat>();  // Wipe, takes memory

  // Give all interest points in a given image a unique id, and put
  // them in a vector with the id corresponding to the interest point
  std::vector<std::map<std::pair<float, float>, int>> keypoint_map(cams.size());
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
      // Initialize to zero for the moment
      keypoint_map[left_index][dist_left_ip] = 0;
      keypoint_map[right_index][dist_right_ip] = 0;
    }
  }
  keypoint_vec.resize(cams.size());
  for (size_t cid = 0; cid < cams.size(); cid++) {
    keypoint_vec[cid].resize(keypoint_map[cid].size());
    int fid = 0;
    for (auto ip_it = keypoint_map[cid].begin(); ip_it != keypoint_map[cid].end();
         ip_it++) {
      auto& dist_ip = ip_it->first;  // alias
      keypoint_map[cid][dist_ip] = fid;
      keypoint_vec[cid][fid] = dist_ip;
      fid++;
    }
  }

  // If feature A in image I matches feather B in image J, which
  // matches feature C in image K, then (A, B, C) belong together in
  // a track, and will have a single triangulated xyz. Build such a track.

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

      int left_id = keypoint_map[left_index][dist_left_ip];
      int right_id = keypoint_map[right_index][dist_right_ip];
      mvg_matches.push_back(openMVG::matching::IndMatch(left_id, right_id));
    }
    match_map[index_pair] = mvg_matches;
  }

  if (verbose) {
    for (auto it = matches.begin(); it != matches.end(); it++) {
      std::pair<int, int> index_pair = it->first;
      dense_map::MATCH_PAIR const& match_pair = it->second;

      int left_index = index_pair.first;
      int right_index = index_pair.second;

      std::string left_image = image_files[left_index];
      std::string right_image = image_files[right_index];

      std::string left_stem = boost::filesystem::path(left_image).stem().string();
      std::string right_stem = boost::filesystem::path(right_image).stem().string();

      std::string match_file = left_stem + "__" + right_stem + ".match";

      std::cout << "Writing: " << left_image << ' ' << right_image << ' '
                << match_file << std::endl;
      dense_map::writeMatchFile(match_file, match_pair.first, match_pair.second);
    }
  }

  // De-allocate data not needed anymore and take up a lot of RAM
  matches.clear(); matches = MATCH_MAP();
  keypoint_map.clear(); keypoint_map.shrink_to_fit();
  cid_to_keypoint_map.clear(); cid_to_keypoint_map.shrink_to_fit();

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
    trackBuilder = openMVG::tracks::TracksBuilder();   // wipe it

    if (map_tracks.empty())
      LOG(FATAL) << "No tracks left after filtering. Perhaps images are too dis-similar?\n";

    // Populate the filtered tracks
    size_t num_elems = map_tracks.size();
    pid_to_cid_fid.resize(num_elems);
    size_t curr_id = 0;
    for (auto itr = map_tracks.begin(); itr != map_tracks.end(); itr++) {
      for (auto itr2 = (itr->second).begin(); itr2 != (itr->second).end(); itr2++) {
        pid_to_cid_fid[curr_id][itr2->first] = itr2->second;
      }
      curr_id++;
    }
  }

  return;
}

}  // end namespace dense_map
