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

#ifndef ISAAC_UTIL_ISAAC_NAMES_H_
#define ISAAC_UTIL_ISAAC_NAMES_H_

// SUBSYSTEMS //////////////////////////////////////////////////////////////////

///////////
// COMMS //
///////////



///////////////
// BEHAVIORS //
///////////////

#define SUBSYSTEM_BEHAVIORS                         "beh"

#define NODE_INSPECTION                             "inspection"
#define TOPIC_BEHAVIORS_INSPECTION_STATE            "beh/inspection/state"
#define SERVICE_BEHAVIORS_INSPECTION_SET_STATE      "beh/inspection/set_state"
#define ACTION_BEHAVIORS_INSPECTION                 "beh/inspection"

#define NODE_CARGO                                  "cargo"
#define TOPIC_BEHAVIORS_CARGO_STATE                 "beh/cargo/state"
#define SERVICE_BEHAVIORS_CARGO_SET_STATE           "beh/cargo/set_state"
#define SERVICE_BEHAVIORS_CARGO_SET_ANOMALY         "beh/cargo/set_anomaly"
#define ACTION_BEHAVIORS_CARGO                      "beh/cargo"

///////////////
// DENSE MAP //
///////////////

#define SUBSYSTEM_DENSE_MAP                         "map"

#define NODE_WIFI_MAPPER                            "wifi_mapper"

#define TOPIC_WIFI_MAPPER_TRACE                     "map/wifi_mapper/trace"
#define TOPIC_WIFI_MAPPER_MAP                       "map/wifi_mapper/vol"

#define NODE_AIR_QUALITY_MAPPER                     "air_quality_mapper"

#define TOPIC_AIR_QUALITY_MAPPER_TRACE              "map/air_quality_mapper/trace"
#define TOPIC_AIR_QUALITY_MAPPER_MAP                "map/air_quality_mapper/vol"

#define NODE_RFID_MAPPER                            "rfid_mapper"

#define TOPIC_RFID_MAPPER_TRACE                     "map/rfid_mapper/trace"
#define TOPIC_RFID_MAPPER_MAP                       "map/rfid_mapper/vol"

#define TOPIC_HEAT_CAM_SIM_POSE                     "sim/heat_cam/pose"
#define TOPIC_HEAT_CAM_SIM_INFO                     "sim/heat_cam/info"
#define TOPIC_ACOUSTICS_CAM_SIM_POSE                "sim/acoustics_cam/pose"
#define TOPIC_ACOUSTICS_CAM_SIM_INFO                "sim/acoustics_cam/info"

////////////////////
// IMG  DETECTION //
////////////////////

#define SUBSYSTEM_ANOMALY                           "det"

#define NODE_IMG_ANALYSIS                           "img_analysis"
#define ACTION_ANOMALY_IMG_ANALYSIS                 "det/img_analysis"

#define NODE_AR_DETECTION                           "ar_detection"
#define ACTION_AR_DETECTION                         "det/ar_detection"

//////////////
// HARDWARE //
//////////////

#define SUBSYSTEM_HARDWARE                          "hw"

#define NODE_WIFI                                   "wifi"

#define TOPIC_HARDWARE_WIFI                         "hw/wifi"
#define TOPIC_WIFI_TRANSMITTER_MAP                  "hw/wifi/transmitter_map"

#define TOPIC_HARDWARE_RFID                         "hw/rfid"

#define TOPIC_HARDWARE_AIR_QUALITY                  "hw/air_quality"

#define SERVICE_HARDWARE_HWDUMMY_SRVNAME            "hw/hwdummy/srvname"

#define TOPIC_HARDWARE_HEAT_CAM                     "hw/cam_heat"
#define TOPIC_HARDWARE_ACOUSTICS_CAM                "hw/cam_acoustics"

///////////////////
// Guest Science //
///////////////////

#define NODE_GS_ACTION_HELPER                       "gs_action_helper"

#define SERVICE_ROS_GS_BRIDGE_GRAB_CONTROL          "ros_gs_bridge/grab_control"

#define TOPIC_GUEST_SCIENCE_IMAGE_INSPECTION_GOAL   "gs/image_inspection/goal"
#define TOPIC_GUEST_SCIENCE_IMAGE_INSPECTION_RESULT "gs/image_inspection/result"
#define TOPIC_GUEST_SCIENCE_INSPECTION_FEEDBACK     "gs/inspection/feedback"
#define TOPIC_GUEST_SCIENCE_INSPECTION_GOAL         "gs/inspection/goal"
#define TOPIC_GUEST_SCIENCE_INSPECTION_RESULT       "gs/inspection/result"

#endif  // ISAAC_UTIL_ISAAC_NAMES_H_
