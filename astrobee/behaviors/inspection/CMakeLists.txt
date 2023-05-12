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
project(inspection)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  std_msgs
  nodelet
  ff_util
  isaac_util
  isaac_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES vent_inspection
  CATKIN_DEPENDS roscpp std_msgs nodelet ff_util isaac_util isaac_msgs
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

# Declare C++ libraries
add_library(inspection
  src/inspection_node.cc
  src/inspection.cc
  src/anomaly_survey.cc
  src/panorama_survey.cc
  src/camera_projection.cc
)
add_dependencies(inspection ${catkin_EXPORTED_TARGETS})
target_link_libraries(inspection ${catkin_LIBRARIES})


## Declare a C++ executable: inspection_tool
add_executable(inspection_tool tools/inspection_tool.cc)
add_dependencies(inspection_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(inspection_tool
  inspection gflags ${catkin_LIBRARIES})

## Declare a C++ executable: sci_cam_tool
add_executable(sci_cam_tool tools/sci_cam_tool.cc)
add_dependencies(sci_cam_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(sci_cam_tool
  gflags ${catkin_LIBRARIES})

## Declare a C++ executable: export_panorama
add_executable(export_panorama tools/export_panorama.cc)
add_dependencies(export_panorama ${catkin_EXPORTED_TARGETS})
target_link_libraries(export_panorama
  inspection gflags ${catkin_LIBRARIES})

## Declare a C++ executable: test_pano
add_executable(test_pano tools/test_pano.cc)
add_dependencies(test_pano ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pano
  inspection gflags ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

# Mark resources files for installation
install(DIRECTORY resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
  PATTERN ".svn" EXCLUDE)

# Required plugin descriptor file for nodelet
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install tools
install(TARGETS inspection_tool
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS sci_cam_tool
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS test_pano
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
