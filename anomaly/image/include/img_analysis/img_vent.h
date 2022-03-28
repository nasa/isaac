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

#ifndef IMG_ANALYSIS_IMG_VENT_H_
#define IMG_ANALYSIS_IMG_VENT_H_

// Standard includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// FSW includes
#include <config_reader/config_reader.h>
#include <ff_util/ff_names.h>
#include <ff_msgs/AckStamped.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>

// Torch include
#include <torch/script.h>

// C++ includes
#include <unsupported/Eigen/CXX11/Tensor>
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <memory>

/**
 * \ingroup beh
 */
namespace img_analysis {

class ImgVent {
 public:
  ImgVent();
  int AnalysePic(cv::Mat img);

  // Vent classification
  static const int vent_free_    = 0;      // Classification of vent as free
  static const int vent_blocked_ = 1;      // Classification of vent as blocked
  static const int vent_unknown_ = 2;      // Classification of vent as unknown

 private:
  torch::jit::script::Module module_;
};
}  // namespace img_analysis
#endif  // IMG_ANALYSIS_IMG_VENT_H_
