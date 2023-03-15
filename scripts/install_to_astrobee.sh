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
#
# usage: install_to_astrobee armhf_dir [ config_version=p4 ]

shopt -s extglob

# Check to see if there are arguments
if [ $# -ne 2 ]; then
  echo "Please supply the armhf folder and target robot as arguments."
  echo "  e.g ./install_to_astrobee.sh ~/freeflyer_armhf_install p4d"
  exit 1
fi

robot_names=(p4c p4d killer bsharp wannabee honey dock queen)

# find which robot is available (or if the specified one is available, if any specified
robot_index=-1
for i in "${!robot_names[@]}"
do
  if [ $# -gt 1 ] && [ $2 != ${robot_names[$i]} ]; then
    continue;
  fi
  robot_index=$i
  break;
done

if [ $robot_index -lt 0 ]; then
  echo "No robot online."
  exit 1
fi

echo "Installing to ${robot_names[$robot_index]}..."
# Remove accidental trailing slashes
self_path=$(dirname "$0")
target=${1%/}
dirname=$(basename ${target})
config_ver=${2:-p4}

echo "$target"

# Some sanity check for the keyboard impaired...
libfile=$target/lib/libinspection.so
if [ ! -f "$libfile" ]; then
    echo "Specified directory does not contain ARS software ready to install!"
    exit 1
fi
if ! readelf -A "$libfile" | grep -q "v7"; then
    echo "Specified directory built for wrong target architecture (not armhf)!"
    exit 1
fi


FREEFLYER_TARGETS=${FREEFLYER_TARGETS=llp mlp}
FREEFLYER_INSTALL_DIR=/opt/isaac/

if [[ ${FREEFLYER_TARGETS,,} =~ 'mlp' ]]; then
  echo "Copying files to MLP..."
  if ! rsync -azh --delete --info=progress2  $target/ astrobee@mlp_iss_${config_ver}:${FREEFLYER_INSTALL_DIR}
  then
    exit 1
  fi
fi

if [ -n "$ip_addr" ]; then
  echo '*** NOTE ***'
  echo 'Be sure to export the following before running anything from this machine: '
  echo "  export ROS_MASTER_URI=http://llp:11311/}"
  echo "  export ROS_HOSTNAME=${ip_addr}"
  echo "  export ROS_SSH_UNKNOWN=1"
fi
