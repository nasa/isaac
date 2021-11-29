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

function apk_version {
  VERSION=$(adb shell dumpsys package $1 | grep versionName)
  if [ $? -eq 0 ]; then
    echo $VERSION
  else
    echo "None Installed"
  fi
}

adb connect hlp

echo "APKs:"
echo "  sci cam image       :  $(apk_version gov.nasa.arc.irg.astrobee.sci_cam_image)"
echo "  guest science bridge:  $(apk_version gov.nasa.arc.irg.astrobee.isaac_gs_ros_bridge)"
