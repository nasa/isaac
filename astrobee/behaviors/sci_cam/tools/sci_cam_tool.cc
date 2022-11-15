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

// Standard ROS includes
#include <ros/ros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>
#include <ff_util/ff_nodelet.h>
#include <isaac_util/isaac_names.h>

// Messages

#define CONVERT_S_TO_MS 0.001

/**
 * \ingroup hw
 */
namespace sci_cam {
class SciCamTool {
 public:
  explicit SciCamTool(ros::NodeHandle *nh) : nh_(nh) {
    Initialize();
  }
  ~SciCamTool() {}

 protected:
  // Called on flight software stack initialization - every NODELET_FATAIL
  // call below should be converted to an initialization fault...
  void Initialize() {
    ROS_ERROR("Running wifi driver node");
    // Read the configuration
    config_reader::ConfigReader config_params;
    // Set the config path to ISAAC
    char *path = getenv("CUSTOM_CONFIG_DIR");
    if (path != NULL)
      config_params.SetPath(path);
    config_params.AddFile("beh/sci_cam.config");
    if (!config_params.ReadFiles()) ROS_FATAL("Couldn't read config file");
  }

 private:
  // ROS variables
  ros::NodeHandle *nh_;
};
}  // namespace sci_cam

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "sci_cam_tool", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  sci_cam::SciCamTool sci_cam_tool(&nh);
  // Synchronous mode
  ros::spin();
  // Make for great success
  return 0;
}
