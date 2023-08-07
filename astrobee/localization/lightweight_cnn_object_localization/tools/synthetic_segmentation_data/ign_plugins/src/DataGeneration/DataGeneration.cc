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

// Ignition Gazebo
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>

// Standard library
#include <iostream>
#include <fstream>
#include <random>
#include <cstdlib>

// Headers
#include "DataGeneration.hh"
#include "../CSVReader/CSVReader.hh"
#include "../SimpleConfigReader/SimpleConfigReader.hh"

IGNITION_ADD_PLUGIN(
    data_generation::DataGeneration,
    ignition::gazebo::System,
    data_generation::DataGeneration::ISystemConfigure,
    data_generation::DataGeneration::ISystemPreUpdate)

std::default_random_engine re;
namespace data_generation {

DataGeneration::DataGeneration() {}
DataGeneration::~DataGeneration() {}

double fRand(double fMax) {
  std::uniform_real_distribution<double> unif(-fMax, fMax);
  return unif(re);
}

ignition::gazebo::components::Pose generateError(ignition::math::Pose3d currentPose) {
  double tEpsilon = 0.15;
  double aEpsilon = 0.07;
  return ignition::gazebo::components::Pose(ignition::math::Pose3d(
    currentPose.Pos().X() + fRand(tEpsilon), currentPose.Pos().Y() + fRand(tEpsilon),
    currentPose.Pos().Z() + fRand(tEpsilon), currentPose.Rot().Euler().X() + fRand(aEpsilon),
    currentPose.Rot().Euler().Y() + fRand(aEpsilon), currentPose.Rot().Euler().Z() + fRand(aEpsilon)));
}

// Run on simulation start
void DataGeneration::Configure(const ignition::gazebo::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                               ignition::gazebo::EntityComponentManager& _ecm,
                               ignition::gazebo::EventManager& _eventMgr) {
  this->entity = _entity;
  this->entityCreator = ignition::gazebo::SdfEntityCreator(_ecm, _eventMgr);

  // Read and load environment variables
  const char* configPathChars = std::getenv("SYNTHETIC_SEGMENTATION_DATA_CONFIG_FILE");
  std::string configPathString(configPathChars);
  const char* scriptPathChars = std::getenv("SYNTHETIC_SEGMENTATION_DATA_SCRIPT_DIR");
  std::string scriptPathString(scriptPathChars);
  const char* outputPathChars = std::getenv("SYNTHETIC_SEGMENTATION_DATA_OUTPUT_DIR");
  std::string outputPathString(outputPathChars);

  // Read and load configuration file
  SimpleConfigReader configReader(configPathString);
  this->NUM_IMAGES_EACH = std::stoi(configReader.getValue("NUM_IMAGES_EACH"));
  this->INSPECTION_POSES_FILEPATH =
    scriptPathString + "/inspection_poses/" + configReader.getValue("INSPECTION_POSES_FILENAME");
  this->inspectionPosesLabels.push_back(configReader.getValue("INSPECTION_POSES_LABEL_NAME"));
  this->inspectionPosesLabels.push_back(configReader.getValue("INSPECTION_POSES_LABEL_INSPECTION_POS_X"));
  this->inspectionPosesLabels.push_back(configReader.getValue("INSPECTION_POSES_LABEL_INSPECTION_POS_Y"));
  this->inspectionPosesLabels.push_back(configReader.getValue("INSPECTION_POSES_LABEL_INSPECTION_POS_Z"));
  this->inspectionPosesLabels.push_back(configReader.getValue("INSPECTION_POSES_LABEL_INSPECTION_ROT_X"));
  this->inspectionPosesLabels.push_back(configReader.getValue("INSPECTION_POSES_LABEL_INSPECTION_ROT_Y"));
  this->inspectionPosesLabels.push_back(configReader.getValue("INSPECTION_POSES_LABEL_INSPECTION_ROT_Z"));
  this->groundTruthLabels.push_back(configReader.getValue("GROUND_TRUTH_LABEL_NAME"));
  this->groundTruthLabels.push_back(configReader.getValue("GROUND_TRUTH_LABEL_POS_X"));
  this->groundTruthLabels.push_back(configReader.getValue("GROUND_TRUTH_LABEL_POS_Y"));
  this->groundTruthLabels.push_back(configReader.getValue("GROUND_TRUTH_LABEL_POS_Z"));
  this->groundTruthLabels.push_back(configReader.getValue("GROUND_TRUTH_LABEL_ROT_X"));
  this->groundTruthLabels.push_back(configReader.getValue("GROUND_TRUTH_LABEL_ROT_Y"));
  this->groundTruthLabels.push_back(configReader.getValue("GROUND_TRUTH_LABEL_ROT_Z"));

  // Read and load inspection names and poses from CSV
  CSVReader csvReader;
  vector<vector<string>> inspectionNameAndPoses = csvReader.readCSV(
    this->INSPECTION_POSES_FILEPATH, this->inspectionPosesLabels);
  for (auto& nameAndPose : inspectionNameAndPoses) {
    this->targetNames.push_back(nameAndPose[0]);
  }
  for (auto& nameAndPose : inspectionNameAndPoses) {
    this->targetInspectPositions.push_back(
      ignition::math::Pose3d(std::stof(nameAndPose[1]), std::stof(nameAndPose[2]), std::stof(nameAndPose[3]),
                             std::stof(nameAndPose[4]), std::stof(nameAndPose[5]), std::stof(nameAndPose[6])));
  }

  // Initialize ground truth output file with column label header if it doesn't already exist
  this->GROUND_TRUTH_FILEPATH = outputPathString + "/groundTruthPoses.csv";
  std::ofstream file(this->GROUND_TRUTH_FILEPATH, std::ios_base::app);
  file << this->groundTruthLabels[0] << "," << this->groundTruthLabels[1] << "," << this->groundTruthLabels[2] << ","
       << this->groundTruthLabels[3] << "," << this->groundTruthLabels[4] << "," << this->groundTruthLabels[5] << ","
       << this->groundTruthLabels[6] << std::endl;
  file.close();
}

// Run on every simulation frame (once every second)
void DataGeneration::PreUpdate(
    const ignition::gazebo::UpdateInfo& _info,
    ignition::gazebo::EntityComponentManager& _ecm) {

  auto sec = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count();
  if (sec > this->lastPositionChange && this->n_count <= targetInspectPositions.size() * this->NUM_IMAGES_EACH) {
    // Randomize camera position
    auto poseComp = _ecm.Component<ignition::gazebo::components::Pose>(this->entity);
    int targetIdx = static_cast<int>(this->n_count / NUM_IMAGES_EACH);
    auto inspectionPoseWithError = generateError(this->targetInspectPositions[targetIdx]);
    *poseComp = inspectionPoseWithError;
    _ecm.SetChanged(
      this->entity, ignition::gazebo::components::Pose::typeId,
      ignition::gazebo::ComponentState::OneTimeChange);
    this->lastPositionChange = sec;

    // Append ground truth pose to file (if statement necessary due to pesky off-by-one bug I don't fully understand)
    if (this->n_count < targetInspectPositions.size() * this->NUM_IMAGES_EACH) {
      std::ofstream file(this->GROUND_TRUTH_FILEPATH, std::ios_base::app);
      file << this->targetNames[targetIdx] << ","
          << inspectionPoseWithError.Data().Pos().X() << "," << inspectionPoseWithError.Data().Pos().Y() << ","
          << inspectionPoseWithError.Data().Pos().Z() << "," << inspectionPoseWithError.Data().Rot().Euler().X() << ","
          << inspectionPoseWithError.Data().Rot().Euler().Y() << "," << inspectionPoseWithError.Data().Rot().Euler().Z()
          << std::endl;
      file.close();
    }

    // Update and print progress
    this->n_count += 1;
    if (static_cast<int>(this->n_count / NUM_IMAGES_EACH) > targetIdx) {
      std::cout << "Target object number " << targetIdx << " is completed. \n";
    }
  }

  // Stop generating data if finished
  if (this->n_count > targetInspectPositions.size() * this->NUM_IMAGES_EACH) {
    this->entityCreator.RequestRemoveEntity(this->entity);
  }
}

}  // namespace data_generation
