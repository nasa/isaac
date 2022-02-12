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

#include <inspection/pano.h>
#include <cmath>

#define EPS 1e-5

namespace inspection {

/* Given panorama parameters specifying desired coverage, write a vector
 * of pan/tilt orientations that will achieve the coverage. Note that
 * this coverage planner always puts the center of the panorama at pan,
 * tilt = 0, 0. But normally these pan/tilt values are interpreted
 * relative to a 6-DOF reference pose, so you can use the reference pose
 * attitude to point the center of the panorama in whatever direction
 * you want.
 *
 * orientations_out: Output PanoAttitude sequence is written to this vector.
 * pan_radius: Cover the pan range -pan_radius .. +pan_radius (radians).
 * tilt_radius: Cover the tilt range -tilt_radius .. +tilt_radius (radians).
 * h_fov: Horizontal FOV of the imager (radians).
 * v_fov: Vertical FOV of the imager (radians).
 * overlap: Required proportion of overlap between consecutive image, 0 .. 1 (unitless).
 * attitude_tolerance: Ensure overlap criterion is met even if relative attitude between a pair of adjacent images, or between an image and the desired pano boundary, is off by at most this much (radians).
 */
void pano_orientations(std::vector<PanoAttitude>* orientations_out,
                       double pan_radius, double tilt_radius,
                       double h_fov, double v_fov,
                       double overlap, double attitude_tolerance) {
  double pan_min = -pan_radius;
  double pan_max = pan_radius;
  double tilt_min = -tilt_radius;
  double tilt_max = tilt_radius;

  // Calculate spacing between pictures
  int ncols = std::ceil((pan_max - pan_min) / (h_fov * (1 - overlap)));
  double k_pan  = (pan_max - pan_min) / ncols;
  int nrows = std::ceil((tilt_max - tilt_min) / (v_fov * (1 - overlap)));
  double k_tilt = (tilt_max - tilt_min) / nrows;

  // Case where max and min is zero
  if (std::isnan(k_pan)) k_pan = M_PI;
  if (std::isnan(k_tilt)) k_tilt = M_PI;

  // If it's a full 360, skip the last one
  if (pan_max - pan_min >= 2 * M_PI) pan_max-= 2 * EPS;  // Go through all the points

  // Generate all the pan/tilt values
  bool top_view = false;
  bool bottom_view = false;
  orientations_out->clear();
  int ix = 0;
  for (double pan = pan_min; pan <= pan_max + EPS; ix++, pan += k_pan) {
    // zig zag through views
    if (ix % 2 == 0) {
      int iy = 0;
      for (double tilt = tilt_min; tilt <= tilt_max + EPS; iy++, tilt += k_tilt) {
        // Avoid taking multiple pictures at top and bottom
        if (tilt < -M_PI/2 + EPS && tilt > -M_PI/2 - EPS) {
          if (bottom_view) {continue;} else {bottom_view = true;}
        }
        if (tilt < M_PI/2 + EPS && tilt > M_PI/2 - EPS) {
          if (top_view) {continue;} else {top_view = true;}
        }
        orientations_out->push_back(PanoAttitude(pan, tilt, iy, ix));
      }
    } else {
      int iy = nrows - 1;
      for (double tilt = tilt_max; tilt >= tilt_min - EPS; iy--, tilt -= k_tilt) {
        // Avoid taking multiple pictures at top and bottom
        if (tilt < -M_PI/2 + EPS && tilt > -M_PI/2 - EPS) {
          if (bottom_view) {continue;} else {bottom_view = true;}
        }
        if (tilt < M_PI/2 + EPS && tilt > M_PI/2 - EPS) {
          if (top_view) {continue;} else {top_view = true;}
        }
        orientations_out->push_back(PanoAttitude(pan, tilt, iy, ix));
      }
    }
  }
}

}  // namespace inspection
