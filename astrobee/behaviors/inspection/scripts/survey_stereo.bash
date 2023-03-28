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
  echo -e "Usage:"
  echo -e "survey_stereo.bash [survey_path] [stereo offset x] [stereo offset y] [stereo offset z]"
  echo
}

case $1 in
    -h | --help )                 print_help
                                  exit
                                  ;;
esac

if [ "$#" -ne 4 ]; then
    echo "** Illegal number of parameters **"
    print_help
    exit
fi


in_traj=$1
out_traj="${in_traj%.*}_stereo.txt"

while read -r line;

do
    stringarray=($line)
    if [ ${stringarray[0]} == "#" ]; then
        echo $line >> $out_traj;
        continue
    fi

    echo "${stringarray[0]} ${stringarray[1]} ${stringarray[2]} ${stringarray[3]} ${stringarray[4]} ${stringarray[5]} ${stringarray[6]}" >> $out_traj;
    echo "$( echo ${stringarray[0]} $2 | awk '{print $1 + $2}' ) $( echo ${stringarray[1]} $3 | awk '{print $1 + $2}' ) $( echo ${stringarray[2]} $4 | awk '{print $1 + $2}' ) ${stringarray[3]} ${stringarray[4]} ${stringarray[5]} ${stringarray[6]}" >> $out_traj;

done < $1