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

function deb_version {
  VERSION=$(dpkg -s $1 2>/dev/null | grep -oP 'Version: \K.+')
  if [ $? -eq 0 ]; then
    echo $VERSION
  else
    echo "None Installed"
  fi
}

source /opt/ros/kinetic/setup.bash
date
echo "Kernel: $(uname -r)"
lsb_release -r
echo "Ros $(rosversion -d) $(rosversion roscpp)"
echo "Debians:"
echo "  isaac0:           $(deb_version isaac0)"
echo "  isaac-config:    $(deb_version isaac-config)"
echo "Modified Files:"
dpkg -V isaac0 | sed 's/^/  /'
dpkg -V isaac-config | sed 's/^/  /'
