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

#include <img_analysis/img_vent.h>

namespace img_analysis {


  ImgVent::ImgVent() {
    // Read the configuration
    config_reader::ConfigReader config_params;
    // Set the config path to ISAAC
    char *path = getenv("CUSTOM_CONFIG_DIR");
    if (path != NULL)
      config_params.SetPath(path);
    config_params.AddFile("anomaly/img_vent.config");
    if (!config_params.ReadFiles()) ROS_FATAL("Couldn't read config file");
    // Get the name of the CNN
    std::string network_name;
    if (!config_params.GetStr("network_name", &network_name))
      ROS_FATAL("Could not find row 'network_name' in table");
    // Set the config path back to freeflyer
    path = getenv("ASTROBEE_CONFIG_DIR");
    if (path != NULL)
      config_params.SetPath(path);

    // Set the model path to to resources
    char *model_path = getenv("ISAAC_CNN_DIR");
    if (model_path == NULL)
      ROS_ERROR("Path containing CNN models not specified");

    // Upload the model
    try {
      // Deserialize the ScriptModule from a file using torch::jit::load().
      module_ = torch::jit::load(std::string(model_path) + "/" + network_name);
    }
    catch (const c10::Error& e) {
      ROS_ERROR_STREAM("error loading the model\n");
      return;
    }
    // ROS_ERROR("ok");
  }

  int ImgVent::AnalysePic(cv::Mat input_img) {
    std::stringstream buffer_print;

    // Resize the input image and transform
    cv::resize(input_img, input_img, cv::Size(244, 244), 0, 0, cv::INTER_CUBIC);

    // Create the tensor
    at::Tensor input_tensor;
    input_tensor = torch::from_blob(input_img.data,
       { 1, input_img.rows, input_img.cols, 3 }, at::kByte).to(at::kFloat);
    input_tensor = input_tensor.permute({ 0, 3, 1, 2 });  // convert to CxHxW

    // Convert data from 0->255 to 0->1
    input_tensor = input_tensor.div(255);

    // Normalize channels
    std::vector<at::Tensor> splits = input_tensor.split(1, 1);

    splits[2] = splits[2].sub(0.485).div(0.229);    // Normalize channel G
    splits[1] = splits[1].sub(0.456).div(0.224);    // Normalize channel B
    splits[0] = splits[0].sub(0.406).div(0.225);    // Normalize channel R
    input_tensor = at::cat({splits[2], splits[1], splits[0]}, 1);

    at::Tensor output = module_.forward({input_tensor}).toTensor();
    output = output.exp();

    return output.argmax().item().to<int64_t>();
  }
}  // namespace img_analysis
