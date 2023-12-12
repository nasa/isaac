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

ARG UBUNTU_VERSION=20.04
ARG REMOTE=isaac
FROM ${REMOTE}/isaac:latest-astrobee-ubuntu${UBUNTU_VERSION}

ARG ROS_VERSION=noetic
ARG PYTHON=3

# suppress detached head warnings later
RUN git config --global advice.detachedHead false

# Install isaac souce dependecies
COPY ./scripts/setup/*.sh /setup/isaac/
COPY ./scripts/setup/dependencies /setup/isaac/dependencies
RUN apt-get update \
  && /setup/isaac/build_install_dependencies.sh \
  && rm -rf /var/lib/apt/lists/*

# Install isaac package dependencies 
COPY ./scripts/setup/packages*.lst /setup/isaac/
RUN /setup/isaac/install_desktop_packages.sh \
  && rm -rf /var/lib/apt/lists/*

# Install acoustics cam dependencies
# RUN apt-get update \
#   && apt-get install -y python${PYTHON}-pip \
#   && pip${PYTHON} install numpy scipy pillow PyWavelets \
#   networkx matplotlib \
#   #scikit-image \
#   pyroomacoustics \
#   && rm -rf /var/lib/apt/lists/*

# Build the code
COPY . /src/isaac/src
RUN export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/include/libtorch/share/cmake/Torch && \
  . /src/astrobee/devel/setup.sh && \
  cd /src/isaac && \
  catkin init && \
  { catkin build || true; } && \
  catkin build

# Add entrypoint
RUN echo "#!/bin/bash\nset -e\n\nsource \"/opt/ros/${ROS_VERSION}/setup.bash\"\nsource \"/src/isaac/devel/setup.bash\"\nexport ASTROBEE_CONFIG_DIR=\"/src/astrobee/src/astrobee/config\"\nexec \"\$@\"" > /ros_entrypoint.sh && \
  chmod +x /ros_entrypoint.sh && \
  rosdep update 2>&1 | egrep -v 'as root|fix-permissions'
