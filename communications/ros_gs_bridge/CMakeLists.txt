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
project(ros_gs_bridge)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin marcos and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ff_msgs
  ff_util
  isaac_msgs
  isaac_hw_msgs
  isaac_util
)

# declare the catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_gs_bridge
  CATKIN_DEPENDS roscpp std_msgs ff_msgs ff_util isaac_msgs isaac_hw_msgs isaac_util
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ libraries

add_library(ros_gs_bridge
  src/ros_gs_bridge.cc
)
add_dependencies(ros_gs_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_gs_bridge ${catkin_LIBRARIES})

# Declare a C++ executable: ros_gs_bridge_node
add_executable(ros_gs_bridge_node
  tools/ros_gs_bridge_node.cc
)
add_dependencies(ros_gs_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_gs_bridge_node ros_gs_bridge ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
