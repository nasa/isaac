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

#include <boost/functional/hash.hpp>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <utility>
#include <unordered_map>
#include "inspection/pano.h"

typedef decltype(&inspection::pano_orientations) orientations_func_t;

class TestCase {
 public:
  std::string label;
  double pan_radius_degrees, tilt_radius_degrees;
  double h_fov_degrees, v_fov_degrees;
  double overlap;
  double plan_attitude_tolerance_degrees;
  double test_attitude_tolerance_degrees;
};

std::string read_string(std::istream* in) {
  std::string result;
  std::getline(*in, result, ',');
  return result.substr(1, result.size() - 2);
}

double read_double(std::istream* in) {
  std::string s;
  std::getline(*in, s, ',');
  return std::stod(s);
}

TestCase read_test_case(std::istream* in) {
  TestCase result;
  result.label = read_string(in);
  result.pan_radius_degrees = read_double(in);
  result.tilt_radius_degrees = read_double(in);
  result.h_fov_degrees = read_double(in);
  result.v_fov_degrees = read_double(in);
  result.overlap = read_double(in);
  result.plan_attitude_tolerance_degrees = read_double(in);
  result.test_attitude_tolerance_degrees = read_double(in);
  return result;
}

void write_pano_csv(const std::string& label,
                    const std::vector<inspection::PanoAttitude>& orientations) {
  std::string out_path("case_" + label + ".csv");
  std::ofstream out_csv(out_path);
  if (!out_csv) {
    std::cerr << "couldn't open " << out_path << " for writing" << std::endl;
  }
  out_csv << "\"pan\",\"tilt\",\"iy\",\"ix\"" << std::endl;

  for (const auto& orient : orientations) {
    out_csv << DEG_FROM_RAD(orient.pan) << ","
            << DEG_FROM_RAD(orient.tilt) << ","
            << orient.iy << ","
            << orient.ix << std::endl;
  }

  out_csv.close();
}

void print_pano(const std::string& label,
                const std::vector<inspection::PanoAttitude>& orientations,
                int nrows, int ncols) {
  std::cout << label << ": ";
  printf("%lu images, %d rows x %d cols, frame# [pan tilt]:\n", orientations.size(), nrows, ncols);

  // build lookup
  inspection::OrientLookupMap orient_lookup;
  inspection::get_orient_lookup(&orient_lookup, orientations);

  // print table format
  for (int iy = 0; iy < nrows; iy++) {
    for (int ix = 0; ix < ncols; ix++) {
      const auto& it = orient_lookup.find(inspection::OrientLookupKey(iy, ix));
      if (it == orient_lookup.end()) {
        printf("                  ");
      } else {
        auto& orient_pair = it->second;
        int frame = orient_pair.first;
        const inspection::PanoAttitude& orient = orient_pair.second;
        printf("%3d [%4ld %4ld]   ",
               frame,
               lrintf(DEG_FROM_RAD(orient.pan)),
               lrintf(DEG_FROM_RAD(orient.tilt)));
      }
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;
}

void do_test_case(orientations_func_t orientations_func, const TestCase& test_case) {
  int nrows, ncols;
  std::vector<inspection::PanoAttitude> orientations;
  orientations_func(&orientations, &nrows, &ncols,
                    RAD_FROM_DEG(test_case.pan_radius_degrees),
                    RAD_FROM_DEG(test_case.tilt_radius_degrees),
                    RAD_FROM_DEG(test_case.h_fov_degrees),
                    RAD_FROM_DEG(test_case.v_fov_degrees),
                    test_case.overlap,
                    RAD_FROM_DEG(test_case.plan_attitude_tolerance_degrees));

  write_pano_csv(test_case.label, orientations);
  print_pano(test_case.label, orientations, nrows, ncols);
}

void do_test_cases(const std::string& label, orientations_func_t orientations_func) {
  std::cout << label << std::endl << std::endl;

  std::string csv_path("pano_test_cases.csv");
  std::ifstream csv_stream(csv_path);
  if (!csv_stream) {
    std::cerr << "couldn't open " << csv_path << " for reading" << std::endl;
    return;
  }

  std::string line;
  std::getline(csv_stream, line);  // ignore header row
  while (!csv_stream.eof()) {
    std::getline(csv_stream, line);
    if (std::string::npos == line.find_first_not_of(" \n")) {
      break;
    }
    std::istringstream row(line);
    TestCase test_case = read_test_case(&row);
    do_test_case(orientations_func, test_case);
  }

  csv_stream.close();
}

int main(int argc, char* argv[]) {
  int which_algo = 2;
  if (argc >= 2) {
    std::string arg = argv[1];
    if (arg == "1") {
      which_algo = 1;
    } else if (arg == "2") {
      which_algo = 2;
    } else {
      std::cerr << "usage: test_pano [1|2]" << std::endl;
      std::cerr << "couldn't parse argument" << std::endl;
    }
  }

  if (which_algo == 1) {
    do_test_cases("=== pano_orientations === ", &inspection::pano_orientations);
  } else {
    do_test_cases("=== pano_orientations2 === ", &inspection::pano_orientations2);
  }
  return 0;
}
