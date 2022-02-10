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

// Anomaly detectors
#include <img_analysis/img_vent.h>

#include <gtest/gtest.h>

#include <dirent.h>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <iostream>

img_analysis::ImgVent vent_analysis_;

int accuracy_ = 0;
int img_counter_ = 0;

void ClassifyFolder(std::string path, int test_class) {
  DIR *dir; struct dirent *diread;
  std::string img_extension = ".jpg";
  int classification = -1;

  if ((dir = opendir(path.c_str())) != nullptr) {
    while ((diread = readdir(dir)) != nullptr) {
      if (std::string(diread->d_name).size() <= img_extension.size() ||
            std::string(diread->d_name).compare(std::string(diread->d_name).size()
            - img_extension.size(), img_extension.size(), img_extension) != 0)
        continue;

      cv::Mat input_img = cv::imread(path + diread->d_name, cv::IMREAD_COLOR);
      classification = vent_analysis_.AnalysePic(input_img);
      img_counter_++;
      if (classification == test_class) {
        accuracy_++;
      }
    }
    closedir(dir);
  } else {
    ROS_ERROR_STREAM("Couldn't open " << path);
    ASSERT_EQ(true, false);
  }
}

TEST(VentTest, VentTest) {
  // Generate testset
  std::string cmd = std::string("rosrun img_analysis get_train_data_vent")
                    + " -vent_poses " + std::string(std::getenv("ISAAC_CNN_DIR")) + "/vent_poses"
                    + " -other_poses " + std::string(std::getenv("ISAAC_CNN_DIR")) + "/other_poses"
                    + " -path_dataset " + std::string(std::getenv("DATA_DIR")) + "vent_dataset"
                    + " -train_pics_per_vent 0"
                    + " -test_pics_per_vent 2";
  std::cout << cmd << std::endl;
  std::system(cmd.c_str());

  std::string path;
  path = std::string(std::getenv("DATA_DIR")) + "vent_dataset/test/free/";
  ClassifyFolder(path, vent_analysis_.vent_free_);

  path = std::string(std::getenv("DATA_DIR")) + "vent_dataset/test/obstacle/";
  ClassifyFolder(path, vent_analysis_.vent_blocked_);

  path = std::string(std::getenv("DATA_DIR")) + "vent_dataset/test/unknown/";
  ClassifyFolder(path, vent_analysis_.vent_unknown_);

  ASSERT_GE(static_cast<float>(accuracy_)/static_cast<float>(img_counter_), 0.8);

  // Remove generated images
  cmd = "rm -r " + std::string(std::getenv("DATA_DIR")) + "vent_dataset/*";
  std::cout << cmd << std::endl;
  std::system(cmd.c_str());
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

