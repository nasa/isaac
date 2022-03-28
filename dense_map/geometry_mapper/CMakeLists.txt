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
project(geometry_mapper)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nodelet
  camera
  interest_point
  sparse_mapping
  ff_util
  msg_conversions
  isaac_hw_msgs
  eigen_conversions
  rosbag
  cv_bridge
  image_transport
  config_reader
  localization_common
)

# The quantity CMAKE_PREFIX_PATH can be set as an environmental
# variable before invoking catkin. Here we append its
# value to CMAKE_MODULE_PATH.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH})
message(STATUS "CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}")
message(STATUS "CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")

# Find OpenCV and fix a 3.3.1 bug
find_package(OpenCV 3 REQUIRED)
if (${OpenCV_VERSION} MATCHES "3.3.1")
  foreach(__cvcomponent ${OpenCV_LIB_COMPONENTS})
    set (__original_cvcomponent ${__cvcomponent})
    if(NOT __cvcomponent MATCHES "^opencv_")
      set(__cvcomponent opencv_${__cvcomponent})
    endif()
    if (TARGET ${__cvcomponent})
      set_target_properties(${__cvcomponent} PROPERTIES
          MAP_IMPORTED_CONFIG_DEBUG ""
          MAP_IMPORTED_CONFIG_RELEASE ""
          MAP_IMPORTED_CONFIG_RELWITHDEBINFO ""
          MAP_IMPORTED_CONFIG_MINSIZEREL ""
      )
    endif()
  endforeach(__cvcomponent)
endif()
set(OpenCV_LIBRARIES ${OpenCV_LIBS})

find_package(OpenMVG QUIET REQUIRED)

find_package(Protobuf REQUIRED)
if (NOT PROTOBUF_PROTOC_EXECUTABLE)
  message(FATAL_ERROR "Could not find system's protoc execuable")
endif (NOT PROTOBUF_PROTOC_EXECUTABLE)
find_package(Ceres REQUIRED)
ADD_DEFINITIONS(-DFREEFLYER_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
find_package(TIFF REQUIRED)
find_package(OpenMP REQUIRED)
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# Build the texrecon external dependency
include(ExternalProject)
set(TEXRECON_DIR ${CMAKE_BINARY_DIR}/texrecon)
ExternalProject_Add(texrecon	
    PREFIX ${TEXRECON_DIR}
    GIT_REPOSITORY https://github.com/oleg-alexandrov/mvs-texturing.git
    GIT_TAG 015a2ac
    CMAKE_ARGS -DCMAKE_VERBOSE_MAKEFILE=TRUE -DCMAKE_CXX_FLAGS=-fPIC
    BUILD_COMMAND $(MAKE)
    # Per: https://cmake.org/pipermail/cmake/2011-April/043772.html,
    # set some options below to not have it always recreate CMakeLists files
    BUILD_ALWAYS 0
    UPDATE_COMMAND ""
    INSTALL_COMMAND cmake -E echo "Skipping install step for texrecon."
    )
    
set(TEXRECON_LIBRARIES
    ${TEXRECON_DIR}/src/texrecon-build/libs/tex/libtex.a
    ${TEXRECON_DIR}/src/texrecon/elibs/mve/libs/mve/libmve.a
    ${TEXRECON_DIR}/src/texrecon/elibs/mve/libs/util/libmve_util.a
    ${JPEG_LIBRARIES} ${PNG_LIBRARIES} ${TIFF_LIBRARIES}
)
set(TEXRECON_INCLUDE_DIRS 
    ${TEXRECON_DIR}/src/texrecon/elibs/rayint/libs
    ${TEXRECON_DIR}/src/texrecon/elibs/mve/libs
    ${TEXRECON_DIR}/src/texrecon/libs
)

# declare the catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geometry_mapper
  CATKIN_DEPENDS roscpp std_msgs nodelet camera interest_point sparse_mapping ff_util isaac_hw_msgs eigen_conversions rosbag cv_bridge image_transport config_reader localization_common
# DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TEXRECON_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OPENMVG_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# Declare a C++ library
add_library(geometry_mapper_lib
    src/dense_map_utils.cc
    src/dense_map_ros_utils.cc
    src/TinyEXIF.cc
    src/interest_point.cc
    src/texture_processing.cc
)

# Specify libraries to link a library or executable target against
target_link_libraries(geometry_mapper_lib
  ${catkin_LIBRARIES}
  ${TEXRECON_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${CERES_LIBRARIES}
  ${OPENMVG_LIBRARIES}
)

# Make sure geometry_mapper_lib is not built before texrecon is done building
add_dependencies(geometry_mapper_lib texrecon)

add_executable(geometry_mapper tools/geometry_mapper.cc)
target_link_libraries(geometry_mapper
    geometry_mapper_lib gflags glog)

add_executable(camera_calibrator tools/camera_calibrator.cc)
target_link_libraries(camera_calibrator
    geometry_mapper_lib gflags glog)

add_executable(camera_refiner tools/camera_refiner.cc)
target_link_libraries(camera_refiner
    geometry_mapper_lib gflags glog)

add_executable(camera_refiner_old tools/camera_refiner_old.cc)
target_link_libraries(camera_refiner_old
    geometry_mapper_lib gflags glog)

add_executable(get_depth_data_from_mesh tools/get_depth_data_from_mesh.cc)
target_link_libraries(get_depth_data_from_mesh
    geometry_mapper_lib gflags glog)

add_executable(extract_pc_from_bag tools/extract_pc_from_bag.cc)
target_link_libraries(extract_pc_from_bag
    geometry_mapper_lib gflags glog)

add_executable(add_sci_cam_to_bag tools/add_sci_cam_to_bag.cc)
target_link_libraries(add_sci_cam_to_bag
    geometry_mapper_lib gflags glog)

add_executable(scale_bag tools/scale_bag.cc)
target_link_libraries(scale_bag
    geometry_mapper_lib gflags glog)

add_executable(colorize_bag_images tools/colorize_bag_images.cc)
target_link_libraries(colorize_bag_images
    geometry_mapper_lib gflags glog)

add_executable(test_correct tools/test_correct.cc)
target_link_libraries(test_correct
    geometry_mapper_lib gflags glog)

add_executable(image_picker tools/image_picker.cc)
target_link_libraries(image_picker
    geometry_mapper_lib gflags glog)

add_executable(orthoproject tools/orthoproject.cc)
target_link_libraries(orthoproject
    geometry_mapper_lib gflags glog)

add_executable(test_texture_gen tools/test_texture_gen.cc)
target_link_libraries(test_texture_gen
    geometry_mapper_lib gflags glog)

add_executable(old_orthoproject tools/old_orthoproject.cc)
target_link_libraries(old_orthoproject
    geometry_mapper_lib gflags glog)

add_executable(streaming_mapper tools/streaming_mapper.cc)
target_link_libraries(streaming_mapper
    geometry_mapper_lib gflags glog)

#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

