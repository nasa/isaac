#!/bin/bash
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

# Exit immediately if a command exits with a non-zero status
set -e

# short help
usage_string="$scriptname [-h]  [-x <use ubuntu 16.04 image>]\
 [-b <use ubuntu 18.04 image>] [-f <use ubuntu 20.04 image>]\
 [-a <astrobee source path>] [-d <idi source path>]\
 [-m <mast source path>] [-n <don't use mast>] [-v <building on a VM>]"
#[-t make_target]

# Print the script usage
print_usage()
{
    echo $usage_string
}

# Print the help message (list all the options)
print_help()
{
    echo "scriptname usage:"
    echo $usage_string
    echo -e "\t-a astrobee_source_path   specify the astrobee source directory to use"
    echo -e "\t   default=isaac_source/../../astrobee"
    echo -e "\t-d idi_source_path   specify the idi source directory to use"
    echo -e "\t   default=isaac_source/../../isaac_data_interface"
    echo -e "\t-m mast_source_path   specify the mast source directory to use"
    echo -e "\t   default=isaac_source/../../mast/src"
    echo -e "\t-n build without MAST"
    echo
}
os=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`
mast=1
vm=0

while [ "$1" != "" ]; do
    case $1 in
        -x | --xenial )               os="xenial"
                                      ;;
        -b | --bionic )               os="bionic"
                                      ;;
        -f | --focal )                os="focal"
                                      ;;
        -a | --astrobee_source_dir )  shift
                                      astrobee_source=$1
                                      ;;
        -d | --idi_source_dir )       shift
                                      idi_source=$1
                                      ;;
        -m | --mast_source_dir )      shift
                                      mast_source=$1
                                      ;;
        -n | --no_mast )              mast=0
                                      ;;
        -v | --vm )                   vm=1
                                      ;;
        -h | --help )                 print_help
                                      exit
                                      ;;
        * )                           print_usage
                                      exit 1
    esac
    shift
done

# Find out isaac root dir
thisdir=$(dirname "$(readlink -f "$0")")
rootdir=$(realpath ${thisdir}/../..)

# Define root dir of different repos
astrobee_source=$(realpath ${astrobee_source:-${rootdir}/../../astrobee/src})
isaac_source=${rootdir}
idi_source=$(realpath ${idi_source:-${rootdir}/../../isaac_data_interface})
if $mast; then
	mast_source=$(realpath ${mast_source:-${rootdir}/../../mast/src})
fi

echo "Using Ubuntu version "${os}
echo "Astrobee path: "${astrobee_source}
echo "ISAAC path: "${isaac_source}
echo "IDI path: "${idi_source}
echo "Build MAST?:" $mast " MAST path: "${mast_source}

UBUNTU_VERSION=16.04
ROS_VERSION=kinetic
PYTHON=''

if [ "$os" = "bionic" ]; then
  UBUNTU_VERSION=18.04
  ROS_VERSION=melodic
  PYTHON=''

elif [ "$os" = "focal" ]; then
  UBUNTU_VERSION=20.04
  ROS_VERSION=noetic
  PYTHON='3'
fi

if [ $vm == 0 ]; then
# Build Astrobee
  docker build ${astrobee_source}/ \
            -f ${astrobee_source}/scripts/docker/astrobee_base.Dockerfile \
            --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
            --build-arg ROS_VERSION=${ROS_VERSION} \
            --build-arg PYTHON=${PYTHON} \
            -t astrobee/astrobee:latest-base-ubuntu${UBUNTU_VERSION}
docker build ${astrobee_source}/ \
            -f ${astrobee_source}/scripts/docker/astrobee.Dockerfile \
            --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
            --build-arg ROS_VERSION=${ROS_VERSION} \
            --build-arg PYTHON=${PYTHON} \
            -t astrobee/astrobee:latest-ubuntu${UBUNTU_VERSION}

# Build ISAAC
docker build ${isaac_source:-${rootdir}} \
            -f ${isaac_source:-${rootdir}}/scripts/docker/isaac_astrobee.Dockerfile \
            --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
            --build-arg ROS_VERSION=${ROS_VERSION} \
            --build-arg PYTHON=${PYTHON} \
            -t isaac/isaac:latest-astrobee-ubuntu${UBUNTU_VERSION}
docker build ${isaac_source:-${rootdir}} \
            -f ${isaac_source:-${rootdir}}/scripts/docker/isaac.Dockerfile \
            --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
            --build-arg ROS_VERSION=${ROS_VERSION} \
            --build-arg PYTHON=${PYTHON} \
            -t isaac/isaac:latest-ubuntu${UBUNTU_VERSION}
fi

# Build messages
docker build ${astrobee_source:-${rootdir}} \
            -f ${isaac_source:-${rootdir}}/scripts/docker/astrobee_msgs.Dockerfile \
            --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
            --build-arg ROS_VERSION=${ROS_VERSION} \
            --build-arg PYTHON=${PYTHON} \
            -t isaac/astrobee:msgs-ubuntu${UBUNTU_VERSION}
docker build ${isaac_source:-${rootdir}} \
            -f ${isaac_source:-${rootdir}}/scripts/docker/isaac_msgs.Dockerfile \
            --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
            --build-arg ROS_VERSION=${ROS_VERSION} \
            --build-arg PYTHON=${PYTHON} \
            -t isaac/isaac:msgs-ubuntu${UBUNTU_VERSION}

# # Build IDI and MAST
# export IDI_PATH=${idi_source}
# export MAST_PATH=${mast_source}
# export UBUNTU_VERSION=${UBUNTU_VERSION}
# export ROS_VERSION=${ROS_VERSION}
# export PYTHON=${PYTHON}

# if [ $mast == 1 ]; then
# 	docker-compose -f ${thisdir}/docker_compose/ros.docker-compose.yml -f ${IDI_PATH}/idi.docker-compose.yml -f ${thisdir}/docker_compose/mast.docker_compose.yml build
# else
# 	docker-compose -f ${thisdir}/docker_compose/ros.docker-compose.yml -f ${IDI_PATH}/idi.docker-compose.yml build
# fi
