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

# This will set up an Astrobee docker container using the non-NASA install instructions.
# You must set the docker context to be the repository root directory

ARG UBUNTU_VERSION=16.04
ARG REMOTE=isaac
FROM ${REMOTE}/isaac:astrobee-msgs-ubuntu${UBUNTU_VERSION}

ARG ROS_VERSION=kinetic
ARG PYTHON=""

# Copy over the isaac_msgs
COPY isaac_msgs /src/msgs/src/

RUN cd /src/msgs \
    && export CMAKE_PREFIX_PATH=/opt/ros/${ROS_VERSION} \
    && catkin init \
    && catkin build
