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

// A little tool to help testing brightness correction

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <dense_map_utils.h>

#include <TinyEXIF.h>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Eigen includes
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <fstream>
#include <iostream>

void test_gamma() {
  std::string filename = "gamma.txt";
  std::cout << "Writing: " << filename << std::endl;

  std::ofstream fh(filename.c_str());
  fh.precision(17);

  int n = 1000;
  for (int col = 0; col < n; col++) {
    double x = static_cast<double>(col) / (n - 1.0);
    double y = gamma(x);
    double z = dense_map::inv_gamma(gamma(x));
    double w = dense_map::inv_gamma(x);

    fh << x << ' ' << y << ' ' << z << ' ' << w << std::endl;
  }
  fh.close();
}

int main(int argc, const char** argv) {
  // test_gamma();
  // return 0;

  std::vector<cv::Mat> images;
  std::vector<double> exposures, isos, products;
  std::vector<std::string> image_files;

  for (int it = 0; it < argc - 1; it++) {
    // read entire image file
    std::string image_file = argv[it + 1];
    image_files.push_back(image_file);
    std::ifstream file(image_file.c_str(), std::ifstream::in | std::ifstream::binary);
    file.seekg(0, std::ios::end);
    std::streampos length = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<uint8_t> data(length);
    file.read(reinterpret_cast<char*>(data.data()), length);

    // parse image EXIF metadata
    TinyEXIF::EXIFInfo imageEXIF(data.data(), length);
    if (imageEXIF.Fields) {
#if 1
      std::cout << "Image Description  " << imageEXIF.ImageDescription << "\n"
                << "Image Resolution   " << imageEXIF.ImageWidth << "x" << imageEXIF.ImageHeight << " pix\n"
                << "Camera Model       " << imageEXIF.Make << " - " << imageEXIF.Model << "\n"
                << "Focal Length       " << imageEXIF.FocalLength << " mm" << std::endl
                << "Exposure (seconds) " << imageEXIF.ExposureTime << std::endl
                << "ISO (int)          " << imageEXIF.ISOSpeedRatings << std::endl
                << "aperture           " << imageEXIF.ApertureValue << std::endl;
#endif

      cv::Mat image;
      images.push_back(image);
      images.back() = cv::imread(image_file);

      // scales.push_back(imageEXIF.ExposureTime * imageEXIF.ISOSpeedRatings);
      exposures.push_back(imageEXIF.ExposureTime);
      isos.push_back(imageEXIF.ISOSpeedRatings);
      products.push_back(imageEXIF.ExposureTime * imageEXIF.ISOSpeedRatings);
    } else {
      LOG(FATAL) << "Could not read info from image: " << image_file << std::endl;
    }
  }

  // return 0;

  if (exposures.empty()) LOG(FATAL) << "No exposures";

  double max_exposure = *std::max_element(exposures.begin(), exposures.end());
  double min_exposure = *std::min_element(exposures.begin(), exposures.end());
  std::cout << "min and max exposure " << min_exposure << ' ' << max_exposure << std::endl;

  double max_iso = *std::max_element(isos.begin(), isos.end());
  double min_iso = *std::min_element(isos.begin(), isos.end());
  std::cout << "min and max iso " << min_iso << ' ' << max_iso << std::endl;

  double max_product = *std::max_element(products.begin(), products.end());
  double min_product = *std::min_element(products.begin(), products.end());
  std::cout << "min and max product " << min_product << ' ' << max_product << std::endl;

  int num_images = images.size();
  for (int it = 0; it < num_images; it++) {
    // double scale = max_iso / isos[it];
    // double scale = max_exposure / exposures[it];
    double scale = max_product / products[it];

    // std::cout << std::endl;
    // std::cout << "input image is " << image_files[it] << std::endl;
    // std::cout << "will scale image in the inv gamma space " << it << " by "
    // << scale << std::endl;

    // Do brightness correction
    cv::Mat out_image;
    double max_iso_times_exposure = 5.2;
    dense_map::exposureCorrection(max_iso_times_exposure, isos[it], exposures[it], images[it], out_image);

    std::ostringstream oss;
    oss << "out_image_test4_" << it + 1000 << ".jpg";
    std::string out_name = oss.str();

    std::cout << "Writing " << out_name << std::endl;
    cv::imwrite(out_name, out_image);
  }

  return 0;
}
