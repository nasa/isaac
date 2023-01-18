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

#ifndef INSPECTION_PANORAMA_SURVEY_H_
#define INSPECTION_PANORAMA_SURVEY_H_

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#define RAD_FROM_DEG(x) ((x) * M_PI / 180.0)
#define DEG_FROM_RAD(x) ((x) * 180.0 / M_PI)

namespace inspection {

class PanoAttitude {
 public:
  double pan;   // radians, 0=forward, increases to the right
  double tilt;  // radians, 0=forward, increases upward
  int16_t iy;   // row index, 0=top row
  int16_t ix;   // column index, 0=left column

  inline PanoAttitude(double _pan, double _tilt, int16_t _iy, int16_t _ix) :
    pan(_pan),
    tilt(_tilt),
    iy(_iy),
    ix(_ix)
  {}
};

typedef std::pair<int, int> OrientLookupKey;
typedef std::pair<int, inspection::PanoAttitude> OrientLookupValue;
typedef std::unordered_map<OrientLookupKey, OrientLookupValue, boost::hash<OrientLookupKey> > OrientLookupMap;

void get_orient_lookup(OrientLookupMap* orient_lookup_out,
                       const std::vector<PanoAttitude>& orientations);

void GeneratePanoOrientations(std::vector<PanoAttitude>* orientations_out,
                        int* nrows_out,
                        int* ncols_out,
                        double pan_radius, double tilt_radius,
                        double h_fov, double v_fov,
                        double overlap, double attitude_tolerance);

}  // namespace inspection

#endif  // INSPECTION_PANORAMA_SURVEY_H_
