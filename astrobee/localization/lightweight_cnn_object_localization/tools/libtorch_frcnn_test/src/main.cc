/* Copyright (c) 2017, United States Government, as represented by the
* Administrator of the National Aeronautics and Space Administration.
* 
* All rights reserved.
* 
* The Astrobee platform is licensed under the Apache License, Version 2.0
* (the "License"); you may not use this file except in compliance with the
* License. You may obtain a copy of the License at
* 
*     http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
* License for the specific language governiing permissions and limitations
* under the License.
*/

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <torchvision/vision.h>

#include <iostream>
#include <memory>
#include <chrono>
#include <string>

int main(int argc, const char* argv[]) {
  if (argc != 3) {
    std::cerr << "usage: libtorch_frcnn_test <path-to-exported-script-module> <path-to-input-image>\n";
    return -1;
  }

  // Load the module
  torch::jit::script::Module module;
  module = torch::jit::load(argv[1]);
  std::cout << "Module loaded OK.\n";

  // Use OpenCV to read the image
  std::string imagePath = argv[2];
  cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
  if (image.empty()) {
      std::cerr << "Error reading image from path: " << imagePath << std::endl;
      return -1;
  }

  // Convert the image to float
  cv::Mat imageFloat;
  image.convertTo(imageFloat, CV_32F, 1.0 / 255.0);

  // Convert the data layout from HxWxC to CxHxW
  torch::Tensor imageTensor = torch::from_blob(imageFloat.data, {image.rows, image.cols, 3}, torch::kFloat32);
  imageTensor = imageTensor.permute({2, 0, 1});

  // Prepare input for testing
  std::vector<torch::Tensor> inputInner;
  inputInner.push_back(imageTensor);
  std::vector<torch::jit::IValue> inputOuter;
  inputOuter.push_back(inputInner);

  // Run the model and report runtime
  auto t_start = std::chrono::high_resolution_clock::now();
  auto output = module.forward(inputOuter).toTuple()->elements()[1].toListRef()[0];
  auto t_end = std::chrono::high_resolution_clock::now();
  std::cout << output << "\n";
  double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
  std::cout << "Module inference ran in " << std::to_string(elapsed_time_ms) << " milliseconds.\n";
}
