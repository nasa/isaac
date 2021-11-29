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
# Declare the project
project(wifi)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED)  # defines catkin_DIR
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  ff_util
  config_reader
  isaac_util
  isaac_hw_msgs
)

# declare the catkin package
catkin_package(
  INCLUDE_DIRS
    include
    ${mnl_INCLUDE_DIRS}
  LIBRARIES
    wifi
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    ff_util
    config_reader
    isaac_util
    isaac_hw_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mnl_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(wifi_lib
  src/wifi.cc
)
target_link_libraries(wifi_lib mnl ${mnl_LDFLAGS})

add_library(wifi
  src/wifi_node.cc
)
add_dependencies(wifi ${catkin_EXPORTED_TARGETS})
target_link_libraries(wifi wifi_lib ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(wifi_tool tools/wifi_tool.cc)
add_dependencies(wifi_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(wifi_tool
  wifi_lib gflags ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS wifi_lib wifi
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS wifi_tool
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

# Required plugin descriptor file for nodelet
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)