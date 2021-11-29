#!/usr/bin/env bash
#
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

export ROS_DISTRO=kinetic

# this script sets the ROS_IP, without it the other processors send back their hostname which can't be resolved
if [ -e /etc/ros.sh ]; then
  source /etc/ros.sh
fi

ros_root=/opt/ros/$ROS_DISTRO
ff_root=/opt/astrobee
isaac_root=/opt/isaac
dds_root=/opt/rti/ndds/lib/armv6vfphLinux3.xgcc4.7.2

#export ASTROBEE_CONFIG_DIR=${ff_root}/config
#export ASTROBEE_RESOURCE_DIR=/res
#export ROSCONSOLE_CONFIG_FILE=$ASTROBEE_RESOURCE_DIR/logging.config

export ISAAC_CONFIG_DIR=/opt/isaac/config
export ISAAC_RESOURCE_DIR==/res

if [ -d /data/ros ]; then
  export ROS_HOME=/data/ros
fi

export ROS_ROOT=${ros_root}/share/ros
export ROS_ETC_DIR=${ros_root}/etc/ros
export ROS_PACKAGE_PATH=${ff_root}/share:${ff_root}/stacks:${isaac_root}/share:${isaac_root}/stacks:${ros_root}/share:${ros_root}/stacks

export LD_LIBRARY_PATH=${ff_root}/lib:${isaac_root}/lib:${dds_root}:${ros_root}/lib:${ros_root}/lib/arm-linux-gnueabihf:/usr/local/lib:/lib:/usr/local/lib:${ff_root}/bin:${isaac_root}/bin
export PATH=${ff_root}/bin:${isaac_root}/bin:${ros_root}/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin
export PYTHONPATH=${ff_root}/lib/python2.7/dist-packages:${isaac_root}/lib/python2.7/dist-packages:${ros_root}/lib/python2.7/dist-packages
export CMAKE_PREFIX_PATH=${ff_root}:${isaac_root}:${ros_root}

unset dds_root
unset ros_root
unset ff_root
unset ip_addr

exec "$@"
