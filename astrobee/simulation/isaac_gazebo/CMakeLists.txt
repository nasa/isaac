# Copyright (c) 2021, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
# platform" software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

cmake_minimum_required(VERSION 3.5)
project(isaac_gazebo)

# Specify C++14 standard
add_definitions(-std=c++14)

# Find packages
find_package(catkin REQUIRED COMPONENTS
    visualization_msgs
    gazebo_ros
    astrobee_gazebo
    sensor_msgs
    ff_msgs
    ff_util
    isaac_util
    isaac_hw_msgs
    isaac_msgs
    cv_bridge
    camera
    jsonloader
)

find_package(gazebo REQUIRED)

# Add gazebo to the link directorries
link_directories(${GAZEBO_LIBRARY_DIRS})

# Call catkin
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    gazebo_sensor_plugin_wifi_receiver
    gazebo_sensor_plugin_wifi_transmitter
    gazebo_sensor_plugin_heat_cam
    gazebo_sensor_plugin_sci_cam
    gazebo_model_plugin_cargo
  CATKIN_DEPENDS
    gazebo_plugins
    gazebo_ros
    astrobee_gazebo
    roscpp
    visualization_msgs
    sensor_msgs
    ff_msgs
    ff_util
    isaac_util
    isaac_msgs
    isaac_hw_msgs
    camera
    jsonloader
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Add gazebo to the link directorries
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Initialise a list which will keep track of all targets
# that need to be installed.
set(targets_to_install "")

# Create a sensor plugin for the wifi receiver
add_library(
    gazebo_sensor_plugin_wifi_receiver
    SHARED src/gazebo_sensor_plugin_wifi_receiver/gazebo_sensor_plugin_wifi_receiver.cc)
target_link_libraries(
    gazebo_sensor_plugin_wifi_receiver
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(
    gazebo_sensor_plugin_wifi_receiver
    ${catkin_EXPORTED_TARGETS})
list(APPEND targets_to_install gazebo_sensor_plugin_wifi_receiver)

# Create a sensor plugin for the wifi transmitter
add_library(
    gazebo_sensor_plugin_wifi_transmitter
    SHARED src/gazebo_sensor_plugin_wifi_transmitter/gazebo_sensor_plugin_wifi_transmitter.cc)
target_link_libraries(
    gazebo_sensor_plugin_wifi_transmitter
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(
    gazebo_sensor_plugin_wifi_transmitter
    ${catkin_EXPORTED_TARGETS})
list(APPEND targets_to_install gazebo_sensor_plugin_wifi_transmitter)

# Create a sensor plugin for the heat cam
add_library(
    gazebo_sensor_plugin_heat_cam
    SHARED src/gazebo_sensor_plugin_heat_cam/gazebo_sensor_plugin_heat_cam.cc)
target_link_libraries(
    gazebo_sensor_plugin_heat_cam
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
    ${OpenCV_LIBRARIES} ${GLOG_LIBRARIES})
add_dependencies(
    gazebo_sensor_plugin_heat_cam
    ${catkin_EXPORTED_TARGETS})
list(APPEND targets_to_install gazebo_sensor_plugin_heat_cam)


# Create a sensor plugin for the RFID receiver
add_library(
    gazebo_sensor_plugin_RFID_receiver
    SHARED src/gazebo_sensor_plugin_RFID_receiver/gazebo_sensor_plugin_RFID_receiver.cc)
target_link_libraries(
    gazebo_sensor_plugin_RFID_receiver
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(
    gazebo_sensor_plugin_RFID_receiver
    ${catkin_EXPORTED_TARGETS})
list(APPEND targets_to_install gazebo_sensor_plugin_RFID_receiver)

# Create a sensor plugin for the RFID transmitter
add_library(
    gazebo_sensor_plugin_RFID_tag
    SHARED src/gazebo_sensor_plugin_RFID_tag/gazebo_sensor_plugin_RFID_tag.cc)
target_link_libraries(
    gazebo_sensor_plugin_RFID_tag
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(
    gazebo_sensor_plugin_RFID_tag
    ${catkin_EXPORTED_TARGETS})
list(APPEND targets_to_install gazebo_sensor_plugin_RFID_tag)

# Create a sensor plugin for the air quality
add_library(
    gazebo_sensor_plugin_air_quality
    SHARED src/gazebo_sensor_plugin_air_quality/gazebo_sensor_plugin_air_quality.cc)
target_link_libraries(
    gazebo_sensor_plugin_air_quality
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
    ${OpenCV_LIBRARIES} ${GLOG_LIBRARIES})
add_dependencies(
    gazebo_sensor_plugin_air_quality
    ${catkin_EXPORTED_TARGETS})
list(APPEND targets_to_install gazebo_sensor_plugin_air_quality)

# Create a sensor plugin for the sci cam
add_library(
  gazebo_sensor_plugin_sci_cam
  SHARED src/gazebo_sensor_plugin_sci_cam/gazebo_sensor_plugin_sci_cam.cc)
target_link_libraries(
  gazebo_sensor_plugin_sci_cam
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(
  gazebo_sensor_plugin_sci_cam
  ${catkin_EXPORTED_TARGETS})
list(APPEND targets_to_install gazebo_sensor_plugin_sci_cam)

# Create a model plugin for the cargo system
add_library(
    gazebo_model_plugin_cargo
    SHARED src/gazebo_model_plugin_cargo/gazebo_model_plugin_cargo.cc)
target_link_libraries(
    gazebo_model_plugin_cargo
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
    ${OpenCV_LIBRARIES} ${GLOG_LIBRARIES})
add_dependencies(
    gazebo_model_plugin_cargo
    ${catkin_EXPORTED_TARGETS})
list(APPEND targets_to_install gazebo_model_plugin_cargo)

#############
## Install ##
#############

# Install all targets that were defined above (some conditionally) and added to the
# variable targets_to_install
install(
  TARGETS
    ${targets_to_install}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  LIBRARY DESTINATION ${LIB_DESTINATION}
)

