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

# Print the help message (list all the options)
print_help()
{
  echo -e "Builds docker images for the ISAAC stack"
  echo -e "The build script will automatically detect the current Ubuntu OS version and"
  echo -e "define the docker files variables UBUNTU_VERSION, ROS_VERSION, and PYTHON accordingly."
  echo -e "Options:"
  echo -e "\t-x | --xenial\t\t\tRun images for Ubuntu 16.04"
  echo -e "\t-b | --bionic\t\t\tRun images for Ubuntu 18.04"
  echo -e "\t-f | --focal\t\t\tRun images for Ubuntu 20.04"
  echo -e "\t-i | --iui-source-dir\t\tSpecify the idi source directory to use"
  echo -e "\t\t\t\tdefault=isaac_source/../../isaac_data_interface"
  echo -e "\t-m | --mast-source-dir\t\tSpecify the mast source directory to use"
  echo -e "\t\t\t\tdefault=isaac_source/../../mast/src"
  echo -e "\t--no-mast\t\t\tDon't run MAST"
  echo -e "\t-vm\t\t\t\tRun the simulation locally - compatible with VM setup"
  echo -e "\t-remote\t\t\t\tRun the remote images (bypass building docker images locally)"
  echo -e "\t-analyst\t\t\tRun analyst notebook"
  echo -e "\t--no-simvm\t\t\tDon't run the simulation"
  echo -e "\t-g | --ground-only\t\tRun only the isaac ground software (hardware in the loop only)"
  echo -e "\t-robot\t\t\t\tSpecify robot name (hardware in the loop only)"
  echo
}

# Print the script usage
print_usage()
{
    echo -e "Invalid option!\n\n"
    print_help
}

# Initialize variables
os=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`
mast=1          # MAST is ON by default
vm=0            # We are not running in a virtual machine by default
remote=0
ground=0
analyst=
robot=bumble
no_sim=0

while [ "$1" != "" ]; do
    case $1 in
        -x | --xenial )               os="xenial"
                                      ;;
        -b | --bionic )               os="bionic"
                                      ;;
        -f | --focal )                os="focal"
                                      ;;
        -d | --iui-source-dir )       shift
                                      iui_source=$1
                                      ;;
        -m | --mast-source-dir )      shift
                                      mast_source=$1
                                      ;;
        --no-mast )                   mast=0
                                      ;;
        -vm )                         vm=1
                                      ;;
        -remote )                     remote=1
                                      ;;
        -analyst )                    analyst=1
                                      ;;
        --no-sim )                    no_sim=1
                                      ;;
        -g | --ground-only )          ground=1
                                      ;;
        -robot )                      shift
                                      robot=$1
                                      ;;
        -h | --help )                 print_help
                                      exit
                                      ;;
        * )                           print_usage
                                      exit 1
    esac
    shift
done

thisdir=$(dirname "$(readlink -f "$0")")
rootdir=${thisdir}/../..

# Define root dir of different repos
isaac_source=${rootdir}/./
iui_source=${iui_source:-${rootdir}/../../isaac_user_interface}
mast_source=${mast_source:-${rootdir}/../../mast/src}
robot=${robot:-bumble}

export ISAAC_PATH=${isaac_source}
export IUI_PATH=${iui_source}
export MAST_PATH=${mast_source}

echo "ISAAC UI path: "${iui_source}
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

if [ $remote -eq 1 ]; then
  export REMOTE=ghcr.io/nasa/
fi

# Launch the rosmaster container + isaac network + IDI
files="-f ${thisdir}/docker_compose/ros.docker-compose.yml -f ${thisdir}/docker_compose/idi.docker-compose.yml"
echo -e "The ISAAC UI is hosted in: http://localhost:8080"
echo -e "The ArangoDB database is hosted in: http://localhost:8529"

# Launch MAST
if [ $mast -eq 1 ]; then
  files+=" -f ${thisdir}/docker_compose/mast.docker_compose.yml"
fi

# Launch the analyst notebook
if [ $analyst -eq 1 ]; then
  files+=" -f ${thisdir}/docker_compose/analyst.docker-compose.yml"
  echo -e "The Analyst Notebook is hosted in: http://localhost:8888/lab?token=isaac"
fi

docker-compose ${files}  up -d

if [ $ground -eq 1 ]; then
  echo "GROUND"
  # Start native astrobee simulation and connect to socket network
  export ROS_IP=`ip -4 addr show docker0 | grep -oP "(?<=inet ).*(?=/)"`
  export ROS_MASTER_URI=http://172.19.0.5:11311
  roslaunch isaac isaac_astrobee.launch llp:=disabled mlp:=disabled ilp:=disabled streaming_mapper:=true output:=screen robot:=${robot} --wait

elif [  $vm -eq 1 ]; then
  echo "VM"
  # Start native astrobee simulation and connect to socket network
  export ROS_IP=`ip -4 addr show docker0 | grep -oP "(?<=inet ).*(?=/)"`
  export ROS_MASTER_URI=http://172.19.0.5:11311
  roslaunch isaac isaac_astrobee.launch llp:=disabled mlp:=disabled ilp:=disabled streaming_mapper:=true output:=screen robot:=${robot} --wait

elif  [ $no_sim -eq 0 ]; then
  echo "NO SIM"
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

fi



