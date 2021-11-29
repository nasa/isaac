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

thisdir=$(dirname "$(readlink -f "$0")")
rootdir=${thisdir}/../../..
export IDI_PATH=${IDI_PATH:-${rootdir}/isaac_data_interface}
export MAST_PATH=${MAST_PATH:-${rootdir}/mast}
cd $thisdir
# docker-compose -f docker-compose.yml -f ${IDI_PATH}/idi.docker-compose.yml -f mast.yml up -d && docker-compose ps
docker-compose -f ${IDI_PATH}/ros.docker-compose.yml -f ${IDI_PATH}/idi.docker-compose.yml up -d && docker-compose ps

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -d --rm --name isaac \
        --network docker_isaac \
        --volume=$XSOCK:$XSOCK:rw \
        --volume=$XAUTH:$XAUTH:rw \
        --volume=`pwd`/simulation.config:/src/astrobee/astrobee/config/simulation/simulation.config:ro \
        --volume=`pwd`/simulation.config:/opt/astrobee/config/simulation/simulation.config:ro \
        --env="XAUTHORITY=${XAUTH}" \
        --env="DISPLAY" \
        --user="astrobee" \
        --env="ROS_MASTER_URI=http://rosmaster:11311" \
        --gpus all \
      isaac \
    /ros_entrypoint.sh roslaunch isaac sim.launch dds:=false wifi:=station streaming_mapper:=true acoustics_cam:=true
