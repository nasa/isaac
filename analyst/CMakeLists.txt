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

# Check to see if correct version of media has been downloaded into tree. We
# dont do this if we are cross-compiling, as there is no need for it. We must
# also make sure the media is copied in a native install to a simulator.

cmake_minimum_required(VERSION 2.8.3)
project(analyst_notebook)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  rosbag
  ff_msgs
	camera
	inspection
	)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosbag
    ff_msgs
	  camera
	  inspection
)


###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable: find_point_coordinate 
add_executable(query_view_points tools/query_view_points.cc)
add_dependencies(query_view_points ${catkin_EXPORTED_TARGETS})
target_link_libraries(query_view_points
gflags glog jsoncpp ${catkin_LIBRARIES})

