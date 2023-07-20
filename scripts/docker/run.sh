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
  echo -e "\t--mast\t\t\tRun MAST"
  echo -e "\t-vm\t\t\t\tRun the simulation locally - compatible with VM setup"
  echo -e "\t--remote\t\t\t\tRun the remote images (bypass building docker images locally)"
  echo -e "\t--analyst\t\t\tRun analyst notebook"
  echo -e "\t--no-sim\t\t\tDon't run the simulation"
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
sim_args="dds:=false robot:=sim_pub wifi:=station"
display="true"
vendor_name=`(lshw -C display | grep vendor) 2>/dev/null`
if [ "$vendor_name" == *"Nvidia"* ]; then
  gpu="true"
else
  gpu="false"
fi
mast=0          # MAST is OFF by default
ground=0        # Ground SW HIL OFF by default
analyst=0
mount="false"
robot=bumble
no_sim=0
local_sim=0
no_iui=0
export REMOTE="isaac"

while [ "$1" != "" ]; do
    case $1 in
        -h | --help )                 print_help
                                      exit
                                      ;;
        -x | --xenial )               os="xenial"
                                      ;;
        -b | --bionic )               os="bionic"
                                      ;;
        -f | --focal )                os="focal"
                                      ;;
        -r | --remote )               export REMOTE="ghcr.io/nasa"
                                      ;;
        -a | --astrobee-source-dir )  shift
                                      astrobee_source=$1
                                      ;;
        -d | --iui-source-dir )       shift
                                      iui_source=$1
                                      ;;
        -m | --mast-source-dir )      shift
                                      mast_source=$1
                                      ;;
        --mast )                      mast=1
                                      ;;
        --analyst )                   analyst=1
                                      ;;
        -n | --no-display )           display="false"
                                      ;;
        -g | --gpu)                   gpu="$2"
                                      shift
                                      ;;
        --mount )                     mount="true"
                                      ;;
        --no-sim )                    no_sim=1
                                      ;;
        --local_sim )                 local_sim=1
                                      ;;
        --no-iui )                    no_iui=1
                                      ;;
        -g | --ground-only )          ground=1
                                      ;;
        -robot )                      shift
                                      robot=$1
                                      ;;
        --args )                      sim_args+=" $2"
                                      shift
                                      ;;
        * )                           print_usage
                                      exit 1
    esac
    shift
done

# collect remaining non-option arguments
cmd="$*"
if [ -z "$cmd" ]; then
    export cmd="roslaunch isaac sim.launch dds:=false robot:=sim_pub $sim_args"
fi

# echo "cmd: $cmd"

if [ "$os" == "xenial" ]; then
  export UBUNTU_VERSION=16.04
  export ROS_VERSION=kinetic
  export PYTHON=''
elif [ "$os" == "bionic" ]; then
  export UBUNTU_VERSION=18.04
  export ROS_VERSION=melodic
  export PYTHON=''
elif [ "$os" == "focal" ]; then
  export UBUNTU_VERSION=20.04
  export ROS_VERSION=noetic
  export PYTHON='3'
else
  echo -e "OS not valid"
  exit 1
fi

######################################################################
# Resolve paths
######################################################################
script_dir=$(dirname "$(readlink -f "$0")")
src_dir=$(dirname $(dirname "$script_dir"))

# Define root dir of different repos
export ASTROBEE_PATH=${astrobee_source:-$(dirname $(dirname "$src_dir"))/astrobee/src}
export ISAAC_PATH=${src_dir}
export IUI_PATH=${iui_source:-$(dirname $(dirname "$src_dir"))/isaac_user_interface}
export MAST_PATH=${mast_source:-$(dirname $(dirname "$src_dir"))/mast/src}

# Print debug variables
echo -e ======================================================================
echo -e "astrobee path: \t "${ASTROBEE_PATH}
echo -e "isaac path: \t "${ISAAC_PATH}
echo -e "isaac ui path: \t "${IUI_PATH}
echo -e "mast path: \t "${MAST_PATH} " \t build mast?:" $mast
echo -e ======================================================================

######################################################################
# Set up docker compose
######################################################################

# Launch the rosmaster container + isaac network + IDI
files="-f ${script_dir}/docker_compose/ros.docker-compose.yml -f ${script_dir}/docker_compose/iui.docker-compose.yml"
echo -e "isaac ui hosted in: \t http://localhost:8080"
echo -e "database hosted in: \t http://localhost:8529"

# Launch MAST
######################################################################
if [ $mast -eq 1 ]; then
  files+=" -f ${script_dir}/docker_compose/mast.docker_compose.yml"
fi

# Launch the analyst notebook
######################################################################
if [ $analyst -eq 1 ]; then
  # Define data locations for analyst notebook
  export DATA_PATH=${HOME}/data
  export BAGS_PATH=$(readlink -f ${HOME}/data/bags)
  export IMG_PATH=/srv/novus_1/mgouveia/data/bags/20220711_Isaac11/

  files+=" -f ${script_dir}/docker_compose/analyst.docker-compose.yml"
  echo -e "analyst notebook hosted in: \t http://localhost:8888/lab?token=isaac"
fi


# Start ground nodes if needed
######################################################################
if [ $ground -eq 1 ]; then
  echo "GROUND SW ONLY (for HIL)"
  # Start native astrobee nodes and connect to socket network
  export ROS_IP=`ip -4 addr show docker0 | grep -oP "(?<=inet ).*(?=/)"`
  export ROS_MASTER_URI=http://172.19.0.5:11311
  robot=${robot:-bumble}
  roslaunch isaac isaac_astrobee.launch llp:=disabled mlp:=disabled ilp:=disabled streaming_mapper:=true output:=screen robot:=${robot} --wait &

elif  [ $local_sim -eq 1 ]; then
  echo "LOCAL SIM"
  # Start native astrobee nodes and connect to socket network
  export ROS_IP=`ip -4 addr show docker0 | grep -oP "(?<=inet ).*(?=/)"`
  export ROS_MASTER_URI=http://172.19.0.5:11311
  robot=${robot:-bumble}
  ${cmd} --wait &

elif  [ $no_sim -eq 1 ]; then
  echo "NO SIM"
  files+=" -f ${script_dir}/docker_compose/astrobee.docker-compose.yml"
  export cmd="roslaunch isaac sim.launch llp:=disabled glp:=disabled gzserver:=false nodes:="framestore,isaac_framestore" output:=screen rviz:=true --wait"

else
# Start isaac robot software on docker container if needed
######################################################################
  files+=" -f ${script_dir}/docker_compose/astrobee.docker-compose.yml"

  # Set up astrobee display
  ######################################################################

  display_args=""
  if [ "$display" = "true" ]; then
      # setup XServer for Docker
      export XSOCK=/tmp/.X11-unix
      export XAUTH=/tmp/.docker.xauth
      touch $XAUTH
      xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

      files+=" -f ${script_dir}/docker_compose/astrobee.docker-compose.display.yml"

      if [ "$gpu" = "true" ]; then
          files+=" -f ${script_dir}/docker_compose/astrobee.docker-compose.gpu.yml"
      fi
  fi

  # Set up astrobee mount
  ######################################################################
  mount_args=""
  if [ "$mount" = "true" ]; then
      files+=" -f ${script_dir}/docker_compose/astrobee.docker-compose.mount.yml"
  fi
fi

echo "${files}"
echo "${cmd}"
# Up compose
######################################################################

echo -e ======================================================================
docker compose ${files} up -d

# eval $cmd

echo -e "Started containers, press 'q' to down container and exit"

# Down compose
######################################################################

count=0
while : ; do
read -n 1 k <&1
if [[ $k = q ]] ; then
printf "\nQuitting from the program\n"
break
else
((count=$count+1))
printf "\nIterate for $count times\n"
echo "Press 'q' to exit"
fi
done

docker compose ${files} down