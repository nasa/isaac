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

cmake_minimum_required(VERSION 2.8.3)
project(vol_mapper)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nodelet
  isaac_util
  isaac_hw_msgs
  config_reader
  tf2_ros
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(gp REQUIRED libgp)

# declare the catkin package
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES vol_mapper
  CATKIN_DEPENDS roscpp std_msgs nodelet isaac_util isaac_hw_msgs config_reader tf2_ros
#  DEPENDS gp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(volumetric_mapper_lib
    src/volumetric_mapper.cc
)
target_link_libraries(volumetric_mapper_lib
  gp ${catkin_LIBRARIES}
)

# Declare a C++ library
add_library(trace_mapper_lib
    src/trace_mapper.cc
)

# Wifi Mapper
add_executable(wifi_mapper
    tools/wifi_mapper_node.cc
)
target_link_libraries(wifi_mapper
  volumetric_mapper_lib trace_mapper_lib ${catkin_LIBRARIES}
)

# Air quality Mapper
add_executable(air_quality_mapper
    tools/air_quality_mapper_node.cc
)
target_link_libraries(air_quality_mapper
  volumetric_mapper_lib trace_mapper_lib ${catkin_LIBRARIES}
)

# RFID Mapper
add_executable(rfid_mapper
    tools/RFID_mapper_node.cc
)
target_link_libraries(rfid_mapper
  volumetric_mapper_lib trace_mapper_lib ${catkin_LIBRARIES}
)
#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
