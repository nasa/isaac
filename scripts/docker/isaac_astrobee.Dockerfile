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
ARG REMOTE=astrobee
FROM ${REMOTE}/astrobee:latest-ubuntu${UBUNTU_VERSION}

# Already inherited from astrobee:base-latest-ubuntu...
ARG ROS_VERSION=kinetic
ARG PYTHON=""

RUN apt-get update && apt-get install -y \
  libmnl-dev \
  ros-${ROS_VERSION}-eigen-conversions \
  ros-${ROS_VERSION}-pcl-ros \
  && rm -rf /var/lib/apt/lists/*

# Minimal isaac robot folders
COPY astrobee /src/isaac/src/astrobee/
COPY isaac /src/isaac/src/isaac/
COPY description /src/isaac/src/description/
COPY isaac_msgs /src/isaac/src/isaac_msgs/
COPY shared /src/isaac/src/shared/

RUN . /src/astrobee/devel/setup.sh \
	&& CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/src/astrobee/devel \
	&& cd /src/isaac \
	&& catkin init \
	&& catkin build