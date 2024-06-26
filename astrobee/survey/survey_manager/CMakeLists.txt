# Copyright (c) 2023, United States Government, as represented by the
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
project(survey_manager)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED)
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  ff_msgs
  lifecycle
  plansys2_executor
)

catkin_package(
  CATKIN_DEPENDS
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

add_library(isaac_action_node src/isaac_action_node.cpp)
# Action for move
add_executable(move_action_node src/move_action_node.cpp)
target_link_libraries(move_action_node isaac_action_node ${catkin_LIBRARIES} )
# Action for dock
add_executable(dock_action_node src/dock_action_node.cpp)
target_link_libraries(dock_action_node isaac_action_node ${catkin_LIBRARIES} )
# Action for undock
add_executable(undock_action_node src/undock_action_node.cpp)
target_link_libraries(undock_action_node isaac_action_node ${catkin_LIBRARIES} )
# Action for panorama
add_executable(panorama_action_node src/panorama_action_node.cpp)
target_link_libraries(panorama_action_node isaac_action_node ${catkin_LIBRARIES} )
# Action for stereo
add_executable(stereo_action_node src/stereo_action_node.cpp)
target_link_libraries(stereo_action_node isaac_action_node ${catkin_LIBRARIES} )

#############
## Install ##
#############

# Allow other packages to use python scripts from this package
catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  tools/command_astrobee
  tools/monitor_astrobee
  tools/pddl_query
  tools/plan_interpreter
  tools/plan_survey
  tools/problem_generator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark executables and/or libraries for installation
install(TARGETS isaac_action_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install actions
install(TARGETS
  move_action_node
  dock_action_node
  undock_action_node
  panorama_action_node
  stereo_action_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach( dir params pddl launch data)
   install( DIRECTORY ${dir}/
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir} )
endforeach(dir)
