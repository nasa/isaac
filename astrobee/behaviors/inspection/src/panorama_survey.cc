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

#include <ros/ros.h>
#include <boost/functional/hash.hpp>
#include <cmath>
#include <string>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include "inspection/panorama_survey.h"

#define EPS 1e-5



/* This library generates panorama surveys given the relevant parameters*/

namespace inspection {



void assert_lte(double a, double b, double eps, const std::string& label) {
  if (!(a <= b + eps)) {
    ROS_ERROR_STREAM("FAIL: at " << label.c_str() << ":" << a << " should be <= " << b << ", within tolerance " << eps);
  }
}

void assert_gte(double a, double b, double eps, const std::string& label) {
  if (!(a >= b - eps)) {
    ROS_ERROR_STREAM("FAIL: at " << label.c_str() << ":" << a << " should be >= " << b << ", within tolerance " << eps);
  }
}

/* Returns the effective HFOV for an image with tilt centered at @tilt.
 * At large tilt values ("high latitude"), effective HFOV is larger
 * because the parallels are shorter near the pole.
 *
 * :param float h_fov: Horizontal field of view of each image (radians).
 * :param float v_fov: Vertical field of view of each image (radians).
 * :param float tilt: The tilt of the image center (radians).
 * :return: The effective HFOV (radians).
 */
double get_h_fov_effective(double h_fov, double v_fov, double tilt) {
  double min_abs_theta = fabs(tilt);
  /* We could take the worst-case HFOV from either the middle, top, or
   * bottom edge of the image, whichever is smallest. This can be
   * important to avoid holes with wide-FOV images near the poles and
   * minimal overlap, but tends to be overly conservative in other
   * cases. That would be delta = 0.5.
   *
   * In this latest version, we'll take an intermediate approach of
   * checking values halfway between the midpoint and the top/bottom,
   * delta = 0.25. Empirically, this seems ok in our coverage validator
   * test cases. */
  double delta = 0.25;

  min_abs_theta = std::min(min_abs_theta, fabs(tilt - delta * v_fov));
  min_abs_theta = std::min(min_abs_theta, fabs(tilt + delta * v_fov));

  return h_fov / cos(min_abs_theta);
}

class LinSpace {
 public:
  double range_min, range_max;
  unsigned int num_pts;
  double step;

  LinSpace(double _range_min, double _range_max, unsigned int _num_pts) :
    range_min(_range_min),
    range_max(_range_max),
    num_pts(_num_pts),
    step((range_max - range_min) / (num_pts - 1))
  {}

  double at(unsigned int i) {
    return range_min + i * step;
  }
  void write_vector(std::vector<double>* pts_out) {
    pts_out->clear();
    double pt = range_min;
    for (int i = 0; i < num_pts; i++) {
      pts_out->push_back(pt);
      pt += step;
    }
  }
  unsigned int find_closest(double val) {
    if (val < range_min) return 0;
    if (val > range_max) return num_pts - 1;
    return lrint((val - range_min) / step);
  }
};

/* Returns image center coordinates such that images cover the range
 * -@range_radius .. +@range_radius with at least the specified
 * @overlap. If one image suffices, it will be centered at 0 (and the
 * edges of the image will extend beyond the range if it is smaller than
 * @fov).  If more than one image is needed to cover the range, the
 * boundary images will cover exactly to the edges of the specified
 * range (modulo @attitude_tolerance) and the images will be evenly
 * spaced.
 *
 * :param range_radius: Images must cover -range_radius .. +range_radius (radians).
 * :param fov: Field of view of each image (radians).
 * :param overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
 * :param attitude_tolerance: Ensure overlap criterion is met even if relative attitude between a pair of adjacent images, or between an image and the desired pano boundary, is off by at most this much (radians).
 * :return: A sequence of orientations of image centers to write to (radians).
 */
LinSpace pano_1d(double range_radius, double fov,
                 double overlap, double attitude_tolerance) {
  double W = range_radius * 2;

  if ((W + 2 * attitude_tolerance - fov) < 0) {
    ROS_ERROR("Special case: Only one image needed. Center it.");
    return LinSpace(0, 0, 1);
  }

  // sufficient overlap criterion: stride <= fov * (1 - overlap) - attitude_tolerance
  // (k - 1) * stride + fov = W + 2 * attitude_tolerance
  // stride = (W + 2 * attitude_tolerance - fov) / (k - 1)
  // (W + 2 * attitude_tolerance - fov) / (k - 1) <= fov * (1 - overlap) - attitude_tolerance
  // k - 1 >= (W + 2 * attitude_tolerance - fov) / (fov * (1 - overlap) - attitude_tolerance)
  // k >= (W + 2 * attitude_tolerance - fov) / (fov * (1 - overlap) - attitude_tolerance) + 1

  int k = std::ceil((W + 2 * attitude_tolerance - fov)
                    / (fov * (1 - overlap) - attitude_tolerance)) + 1;

#if 1
  // optional sanity checks

  if (k > 1) {
    double stride = (W + 2 * attitude_tolerance - fov) / (k - 1);
    assert_lte(stride, fov * (1 - overlap) - attitude_tolerance, EPS, "sufficient overlap");
  }

  // check if we have more images than necessary
  if (k == 1) {
    // obviously need at least one image
  } else if (k == 2) {
    assert_lte(fov, W + 2 * attitude_tolerance, EPS, "k = 1 is not enough");
  } else {
    double stride1 = (W + 2 * attitude_tolerance - fov) / (k - 2);
    assert_gte(stride1, fov * (1 - overlap) - attitude_tolerance, EPS, "k is minimized");
  }
#endif

  double min_center = -(range_radius + attitude_tolerance) + fov / 2;
  double max_center = -min_center;
  return LinSpace(min_center, max_center, k);
}

/* Returns image center coordinates such that the images cover the full pan
 * range -180 .. 180 with at least the specified @overlap between all consecutive images,
 * including at the wrap-around, and the image sequence is centered at pan = 0.
 *
 * :param float fov: Field of view of each image (radians).
 * :param float overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
 * :param float attitude_tolerance: Ensure overlap criterion is met even if relative attitude between a pair of adjacent images, or between an image and the desired pano boundary, is off by at most this much (degrees).
 * :return: A sequence of orientations of image centers (radians).
 */
LinSpace pano_1d_complete_pan(double fov, double overlap, double attitude_tolerance) {
  unsigned int k = std::ceil((2 * M_PI) / (fov * (1 - overlap) - attitude_tolerance));
  double stride = (2 * M_PI / k);
  return LinSpace(-M_PI + 0.5 * stride, M_PI - 0.5 * stride, k);
}

/* Returns image center coordinates for a row of images at tilt value
 * @tilt that cover the range -@pan_radius .. +@pan_radius with at least
 * the specified @overlap. Special complete wrap-around behavior is
 * triggered when @pan_radius is exactly M_PI.
 *
 * :param pan_radius: Cover pan angle of -pan_radius to +pan_radius (radians).
 * :param h_fov: Horizontal field of view of each image (radians).
 * :param v_fov: Vertical field of view of each image (radians).
 * :param overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
 * :param attitude_tolerance: Ensure complete coverage and sufficient overlap even if relative attitude between a pair of adjacent images, or between an image and the desired pano boundary, is off by at most this much (radians).
 * :param tilt: The tilt of the image center (radians).
 * :return: A sequence of pan orientations of image centers (radians).
 */
LinSpace pano_1d_pan(double pan_radius,
                     double h_fov, double v_fov,
                     double overlap, double attitude_tolerance,
                     double tilt) {
  double h_fov_effective = get_h_fov_effective(h_fov, v_fov, tilt);
  if (fabs(pan_radius - M_PI) < EPS) {
    return pano_1d_complete_pan(h_fov_effective, overlap, attitude_tolerance);
  } else {
    return pano_1d(pan_radius, h_fov_effective, overlap, attitude_tolerance);
  }
}

void get_orient_lookup(OrientLookupMap* orient_lookup_out,
                       const std::vector<PanoAttitude>& orientations) {
  orient_lookup_out->clear();
  int frame = 0;
  for (const auto& orient : orientations) {
    orient_lookup_out->insert(std::make_pair(OrientLookupKey(orient.iy, orient.ix),
                                             OrientLookupValue(frame, orient)));
    frame++;
  }
}

/* Given panorama parameters specifying desired coverage, write a vector
 * of pan/tilt orientations that will achieve the coverage. Note that
 * this coverage planner always puts the center of the panorama at pan,
 * tilt = 0, 0. But normally these pan/tilt values are interpreted
 * relative to a 6-DOF reference pose, so you can use the reference pose
 * attitude to point the center of the panorama in whatever direction
 * you want. Note that this algorithm can not guarantee it will satisfy
 * the complete coverage and overlap requirements due to the fact that
 * image coverage areas are warped into non-rectangles in a way that's
 * difficult to handle in general. Therefore, you should validate that
 * the pano covers the area you want by adding your parameters to
 * pano_test_cases.csv, then running test_pano and plot_pano.py. If
 * necessary, encourage the planner to add more images by adding some
 * extra "warp margin" into plan_attitude_tolerance_degrees while
 * leaving alone test_attitude_tolerance_degrees, until the problem is
 * resolved.
 *
 * orientations_out: Write output PanoAttitude sequence here.
 * nrows_out: Write number of rows in panorama here.
 * ncols_out: Write number of columns in panorama here.
 * pan_radius: Cover the pan range -pan_radius .. +pan_radius (radians).
 * tilt_radius: Cover the tilt range -tilt_radius .. +tilt_radius (radians).
 * h_fov: Horizontal FOV of the imager (radians).
 * v_fov: Vertical FOV of the imager (radians).
 * overlap: Required proportion of overlap between consecutive image, 0 .. 1 (unitless).
 * attitude_tolerance: Try to ensure complete coverage and sufficient overlap even if relative attitude between a pair of adjacent images, or between an image and the desired pano boundary, is off by at most this much (radians).
 */
void GeneratePanoOrientations(std::vector<PanoAttitude>* orientations_out,
                        int* nrows_out,
                        int* ncols_out,
                        double pan_radius, double tilt_radius,
                        double h_fov, double v_fov,
                        double overlap, double attitude_tolerance) {
  // calculate all image centers
  std::vector<PanoAttitude> image_centers;
  LinSpace tilt_seq = pano_1d(tilt_radius, v_fov, overlap, attitude_tolerance);
  std::vector<double> tilt_vals;
  tilt_seq.write_vector(&tilt_vals);
  int nrows = tilt_vals.size();
  std::reverse(tilt_vals.begin(), tilt_vals.end());  // order top-to-bottom
  std::vector<double> pan_vals;
  double min_abs_tilt = fabs(tilt_vals[0]);
  int iy = 0;
  for (auto tilt : tilt_vals) {
    LinSpace pan_seq = pano_1d_pan(pan_radius, h_fov, v_fov, overlap, attitude_tolerance, tilt);
    pan_seq.write_vector(&pan_vals);
    min_abs_tilt = std::min(min_abs_tilt, fabs(tilt));
    for (auto pan : pan_vals) {
      image_centers.push_back(PanoAttitude(pan, tilt, iy, -1));
    }
    iy++;
  }

  // assign image centers to columns
  LinSpace col_centers = pano_1d_pan(pan_radius,
                                     h_fov, v_fov,
                                     overlap, attitude_tolerance,
                                     min_abs_tilt);
  unsigned int ncols = col_centers.num_pts;
  for (auto& orient : image_centers) {
    orient.ix = col_centers.find_closest(orient.pan);
  }

  // Do simple greedy column reassignments as needed. This empirically
  // reduces large attitude changes. The principle is to look in the
  // adjacent row that is nearer to the equator (if there is one), find
  // the frame with the closest pan value, and choose the same column it
  // did.
  OrientLookupMap orient_lookup;
  bool changed = true;
  while (changed) {
    changed = false;
    get_orient_lookup(&orient_lookup, image_centers);

    for (auto& orient : image_centers) {
      int ny;
      if (orient.tilt < 0) {
        ny = orient.iy - 1;
      } else {
        ny = orient.iy + 1;
      }
      if (!(0 <= ny && ny < nrows)) {
        continue;
      }

      double min_abs_pan_diff = 999;
      int min_ix = -1;

      for (int nx = orient.ix - 1; nx < orient.ix + 2; nx++) {
        int nx_mod = (nx + ncols) % ncols;
        const auto& nit = orient_lookup.find(OrientLookupKey(ny, nx_mod));
        if (nit != orient_lookup.end()) {
          const inspection::PanoAttitude& n_orient = nit->second.second;
          if (fabs(n_orient.tilt) > fabs(orient.tilt)) {
            // only do reassignment if neighbor row is actually closer to equator
            break;
          }
          double abs_diff0 = fabs(orient.pan - n_orient.pan);
          double abs_diff = std::min(abs_diff0, 360 - abs_diff0);
          if (abs_diff < min_abs_pan_diff) {
            min_abs_pan_diff = abs_diff;
            min_ix = n_orient.ix;
          }
        }
      }
      if (min_ix != -1 && min_ix != orient.ix) {
        orient.ix = min_ix;
        changed = true;
      }
    }
  }

  // order images in column-major order; alternate direction top-to-bottom or bottom-to-top
  orientations_out->clear();
  std::vector<PanoAttitude> images_ordered;
  std::vector<PanoAttitude> col;
  for (int ix = 0; ix < ncols; ix++) {
    col.clear();
    for (const auto& orient : image_centers) {
      // get images in column ix
      if (orient.ix == ix) {
        col.push_back(orient);
      }
    }

    // on odd-numbered columns, reverse tilt order, bottom-to-top
    if (ix % 2 == 1) {
      std::reverse(col.begin(), col.end());
    }

    // append column to output
    orientations_out->insert(orientations_out->end(), col.begin(), col.end());
  }

  (*nrows_out) = tilt_vals.size();
  (*ncols_out) = ncols;
}

}  // namespace inspection
