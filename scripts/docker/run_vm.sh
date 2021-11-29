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

set -e

# short help
usage_string="$scriptname [-h] [-a <astrobee source path>] [-i <isaac source path>] [-d <idi source path>]"
#[-t make_target]

usage()
{
    echo "usage: sysinfo_page [[[-a] [-i] [-d] [-m] [-n]] | [-h]]"
}
mast=1

while [ "$1" != "" ]; do
    case $1 in
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
        -h | --help )                 usage
                                      exit
                                      ;;
        * )                           usage
                                      exit 1
    esac
    shift
done

# Find out isaac root dir
thisdir=$(dirname "$(readlink -f "$0")")
rootdir=${thisdir}/../..

# Define root dir of different repos
isaac_source=${rootdir}/./
idi_source=${idi_source:-${rootdir}/../../isaac_data_interface}
mast_source=${mast_source:-${rootdir}/../../mast/src}

export IDI_PATH=${idi_source}
export MAST_PATH=${mast_source}

echo "IDI path: "${idi_source}
echo "Build MAST?:" $mast " MAST path: "${mast_source}

# Launch the rosmaster container + isaac network + IDI
if [ $mast == 1 ]; then
  docker-compose -f ${thisdir}/docker_compose/ros.docker-compose.yml -f ${thisdir}/docker_compose/idi.docker-compose.yml -f ${thisdir}/docker_compose/mast.docker_compose.yml up -d
else
  docker-compose -f ${thisdir}/docker_compose/ros.docker-compose.yml -f ${thisdir}/docker_compose/idi.docker-compose.yml up -d
fi

# Start native astrobee simulation and connect to socket network
export ROS_IP=`ip -4 addr show docker0 | grep -oP "(?<=inet ).*(?=/)"`
export ROS_MASTER_URI=http://172.19.0.5:11311
roslaunch isaac sim.launch rviz:=true dds:=false robot:=sim_pub ns:=queen streaming_mapper:=true --wait
