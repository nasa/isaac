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

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# if [ -n "$(git status --porcelain)" ]; then 
#   echo "You should not build Debians for a dirty source tree!"
#   echo "Make sure all your changes are committed AND pushed to the server..."
#   exit -1
# fi

EXTRA_FLAGS="-b -a armhf"
if [[ $* == *--config* ]]; then
  EXTRA_FLAGS="-A"
fi

pushd $DIR/../..
export CMAKE_TOOLCHAIN_FILE=${DIR}/isaac_cross.cmake
DEB_BUILD_OPTIONS="parallel=8" debuild -e ARMHF_CHROOT_DIR -e ARMHF_TOOLCHAIN -e CMAKE_TOOLCHAIN_FILE -e CMAKE_PREFIX_PATH -us -uc $EXTRA_FLAGS
popd > /dev/null
