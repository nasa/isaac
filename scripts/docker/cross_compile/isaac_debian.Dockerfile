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

FROM astrobee/astrobee:cross

# Copy the folders around
RUN ln -s /arm_cross/rootfs/opt/ros /opt/ros \
	&& ln -s /opt/astrobee /arm_cross/rootfs/opt/astrobee \
	&& ln -s /arm_cross/rootfs/lib/arm-linux-gnueabihf /lib/arm-linux-gnueabihf \
	&& ln -s /arm_cross/rootfs/usr/lib/arm-linux-gnueabihf /usr/lib/arm-linux-gnueabihf \
	&& ln -s /arm_cross/rootfs/usr/lib/libPocoFoundation.so /usr/lib/libPocoFoundation.so

RUN apt-get update && apt-get install -y \
    devscripts \
    debhelper \
  && rm -rf /var/lib/apt/lists/*
  
# Copy the source code
COPY . /src/isaac_ws/src

# Configure catkin workspace and compile
RUN cd /src/isaac_ws \
	&& export ARMHF_CHROOT_DIR=/arm_cross/rootfs \
	&& export ARMHF_TOOLCHAIN=/arm_cross/toolchain/gcc \
	&& export CMAKE_TOOLCHAIN_FILE=/src/isaac_ws/src/scripts/build/isaac_cross.cmake \
	&& export CMAKE_PREFIX_PATH=/opt/astrobee:/arm_cross/rootfs/opt/ros/kinetic:/src/astrobee/cmake \
	&& ln -s /arm_cross/toolchain/gcc/bin/arm-linux-gnueabihf-strip "/usr/bin/arm-linux-gnueabihf-strip" \
	&& ./src/scripts/build/build_debian.sh

# Move resulting files to a folder
RUN mkdir /src/isaac_ws/debians \
 && mv -t /src/isaac_ws/debians /src/isaac_ws/*.deb /src/isaac_ws/*.build /src/isaac_ws/*.changes
