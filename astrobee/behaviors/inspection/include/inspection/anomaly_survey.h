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

#ifndef INSPECTION_ANOMALY_SURVEY_H_
#define INSPECTION_ANOMALY_SURVEY_H_

// Standard ROS includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// TF2 support
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Shared project includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_service.h>
#include <ff_util/ff_fsm.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>
#include <config_reader/config_reader.h>
#include <ff_util/ff_flight.h>
#include <isaac_util/isaac_names.h>

// Point Cloud
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// Software messages
#include <visualization_msgs/MarkerArray.h>
#include <isaac_msgs/InspectionState.h>
#include <ff_msgs/Zone.h>
#include <geometry_msgs/PoseArray.h>

// Services
#include <ff_msgs/SetState.h>
#include <ff_msgs/GetZones.h>
#include <ff_msgs/GetMap.h>

// Actions
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/DockAction.h>
#include <isaac_msgs/ImageInspectionAction.h>
#include <isaac_msgs/InspectionAction.h>

// Eigen for math
#include <Eigen/Dense>

// C++ headers
#include <vector>

/**
 * \ingroup beh
 */
namespace inspection {



// class CameraView {
//   CameraView(std::string cam_name);
  bool getProjectionMatrix();
  bool PositionToXY(geometry_msgs::Pose point, int &x, int &y);
  // private:
  bool setProjectionMatrix();


// }




}  // namespace inspection

#endif  // INSPECTION_ANOMALY_SURVEY_H_
