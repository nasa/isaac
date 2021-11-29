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
RUN cp -R /arm_cross/rootfs/opt/ros /opt/ros \
	&& cp -R /opt/astrobee /arm_cross/rootfs/opt/astrobee \
	&& cp -R /arm_cross/rootfs/lib/arm-linux-gnueabihf /lib/arm-linux-gnueabihf \
	&& cp -R /arm_cross/rootfs/usr/lib/arm-linux-gnueabihf /usr/lib/arm-linux-gnueabihf \
	&& cp /arm_cross/rootfs/usr/lib/libPocoFoundation.so /usr/lib/libPocoFoundation.so

# Copy the source code
COPY . /src/isaac_ws/src

# Configure catkin workspace and compile
RUN cd /src/isaac_ws \
	&& catkin init \
	&& catkin profile add cross \
	&& catkin profile set cross \
	&& export ARMHF_CHROOT_DIR=/arm_cross/rootfs \
	&& export ARMHF_TOOLCHAIN=/arm_cross/toolchain/gcc \
	&& export CMAKE_TOOLCHAIN_FILE=/src/isaac_ws/src/scripts/build/isaac_cross.cmake \
	&& export CMAKE_PREFIX_PATH=/opt/astrobee:/arm_cross/rootfs/opt/ros/kinetic:/src/astrobee/cmake \
	&& catkin config \
		--install-space /opt/isaac \
		--install \
		--whitelist isaac_astrobee_description isaac_util isaac_msgs inspection isaac_hw_msgs wifi isaac gs_action_helper\
		--cmake-args \
			-DCMAKE_TOOLCHAIN_FILE=/src/isaac_ws/src/scripts/build/isaac_cross.cmake \
			-DARMHF_CHROOT_DIR=/arm_cross/rootfs \
			-DARMHF_TOOLCHAIN=/arm_cross/toolchain/gcc \
	&& catkin build \
	&& chmod +x /opt/isaac/env_wrapper.sh
