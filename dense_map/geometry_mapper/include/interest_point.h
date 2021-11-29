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

#ifndef INTEREST_POINT_H_
#define INTEREST_POINT_H_

#include <ff_common/thread.h>

#include <opencv2/imgproc.hpp>

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <mutex>
#include <utility>

namespace dense_map {

/// A class for storing information about an interest point
/// in the format the NASA ASP can read it (very useful
// for visualization)
struct InterestPoint {
  typedef std::vector<float> descriptor_type;
  typedef descriptor_type::iterator iterator;
  typedef descriptor_type::const_iterator const_iterator;

  // The best way of creating interest points is:
  // ip = InterestPoint(x, y). But in the worst case, when the user
  // chooses to simply create a blank interest point, like
  // InterestPoint ip;
  // at least make sure that its members are always initialized,
  // as seen in the constructor below.
  // TODO(Oleg): There is no way to enforce that ix be in sync with x or
  // iy with y.
  InterestPoint(float x = 0, float y = 0, float scale = 1.0, float interest = 0.0, float ori = 0.0, bool pol = false,
                uint32_t octave = 0, uint32_t scale_lvl = 0)
      : x(x),
        y(y),
        scale(scale),
        ix(int32_t(x)),
        iy(int32_t(y)),
        orientation(ori),
        interest(interest),
        polarity(pol),
        octave(octave),
        scale_lvl(scale_lvl) {}

  /// Subpixel (col, row) location of point
  float x, y;

  /// Scale of point.  This may come from the pyramid level, from
  /// interpolating the interest function between levels, or from some
  /// other scale detector like the Laplace scale used by Mikolajczyk & Schmid
  float scale;

  /// Integer location (unnormalized), mainly for internal use.
  int32_t ix;
  int32_t iy;

  /// Since the orientation is not necessarily unique, we may have more
  /// than one hypothesis for the orientation of an interest point.  I
  /// considered making a vector of orientations for a single point.
  /// However, it is probably better to make more than one interest
  /// point with the same (x,y,s) since the descriptor will be unique
  /// for a given orientation anyway.
  float orientation;

  /// The interest measure (could be Harris, LoG, etc.).
  float interest;

  /// These are some extras for SURF-like implementations
  bool polarity;
  /// This is the integer location in scale space (used for indexing
  /// a vector of interest images)
  uint32_t octave, scale_lvl;

  /// And finally the descriptor for the interest point.  For example,
  /// PCA descriptors would have a vector of floats or doubles...
  descriptor_type descriptor;

  const_iterator begin() const { return descriptor.begin(); }
  iterator begin() { return descriptor.begin(); }
  const_iterator end() const { return descriptor.end(); }
  iterator end() { return descriptor.end(); }

  size_t size() const { return descriptor.size(); }
  float operator[](int index) { return descriptor[index]; }

  /// std::sort can be used to sort InterestPoints in descending
  /// order of interest.
  bool operator<(const InterestPoint& other) const { return (other.interest < interest); }

  /// Generates a human readable string
  std::string to_string() const {
    std::stringstream s;
    s << "IP: (" << x << "," << y << ")  scale: " << scale << "  orientation: " << orientation
      << "  interest: " << interest << "  polarity: " << polarity << "  octave: " << octave
      << "  scale_lvl: " << scale_lvl << "\n";
    s << "  descriptor: ";
    for (size_t i = 0; i < descriptor.size(); ++i) s << descriptor[i] << "  ";
    s << std::endl;
    return s.str();
  }

  /// Copy IP information from an OpenCV KeyPoint object.
  void setFromCvKeypoint(Eigen::Vector2d const& key, cv::Mat const& cv_descriptor) {
    x = key[0];
    y = key[1];
    ix = round(x);
    iy = round(y);
    interest = 0;
    octave = 0;
    scale_lvl = 1;
    scale = 1;
    orientation = 0;
    polarity = false;

    if (cv_descriptor.rows != 1 || cv_descriptor.cols < 2)
      LOG(FATAL) << "The descriptors must be in one row, and have at least two columns.";

    descriptor.resize(cv_descriptor.cols);
    for (size_t it = 0; it < descriptor.size(); it++) {
      descriptor[it] = cv_descriptor.at<float>(0, it);
    }
  }
};  // End class InterestPoint

typedef std::pair<std::vector<InterestPoint>, std::vector<InterestPoint> > MATCH_PAIR;

void detectFeatures(const cv::Mat& image, bool verbose,
                    // Outputs
                    cv::Mat* descriptors, Eigen::Matrix2Xd* keypoints);

// This really likes haz cam first and nav cam second
void matchFeatures(std::mutex* match_mutex, int left_image_index, int right_image_index,
                   cv::Mat const& left_descriptors, cv::Mat const& right_descriptors,
                   Eigen::Matrix2Xd const& left_keypoints, Eigen::Matrix2Xd const& right_keypoints, bool verbose,
                   // Output
                   MATCH_PAIR* matches);

// Routines for reading & writing interest point match files
void writeMatchFile(std::string match_file, std::vector<InterestPoint> const& ip1,
                    std::vector<InterestPoint> const& ip2);

void saveImagesAndMatches(std::string const& left_prefix, std::string const& right_prefix,
                          std::pair<int, int> const& index_pair, MATCH_PAIR const& match_pair,
                          std::vector<cv::Mat> const& images);

}  // namespace dense_map

#endif  // INTEREST_POINT_H_
