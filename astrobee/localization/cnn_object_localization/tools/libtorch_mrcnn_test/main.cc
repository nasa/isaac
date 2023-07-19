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

#include <torch/script.h>
// #include <torch/torch.h>  // Uncomment if there are any issues with linking...?
#include <torchvision/vision.h>
// #include <torchvision/ops/nms.h>  // Uncomment if there are any issues with linking...?

#include <iostream>
#include <memory>
#include <chrono>
#include <string>

int main(int argc, const char* argv[]) {
  if (argc != 2) {
    std::cerr << "usage: libtorch_mrcnn_test <path-to-exported-script-module>\n";
    return -1;
  }
  // if (argc != 3) {
  //   std::cerr << "usage: libtorch_mrcnn_test <path-to-exported-script-module> <path-to-image-file>\n";
  //   return -1;
  // }

  // This line doesn't do anything, but it makes sure the C++ linker doesn't prune libtorchvision
  vision::cuda_version();

  // Load the module
  torch::jit::script::Module module;
  module = torch::jit::load(argv[1]);
  std::cout << "Module loaded OK.\n";

  // Prepare dummy input for testing
  std::vector<torch::Tensor> inputInner;
  inputInner.push_back(torch::ones({3, 240, 320}));
  std::vector<torch::jit::IValue> inputOuter;
  inputOuter.push_back(inputInner);

  // Run the model and report runtime
  auto t_start = std::chrono::high_resolution_clock::now();
  auto output = module.forward(inputOuter).toTuple()->elements()[1];
  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
  std::cout << "Module inference ran in " << std::to_string(elapsed_time_ms) << " milliseconds.\n";
}
