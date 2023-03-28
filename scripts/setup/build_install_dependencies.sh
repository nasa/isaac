#/bin/bash -e
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
# Install the dependencies needed for the debians. Build and install flight
# software debians.

DEP_LOC=$(dirname "$(readlink -f "$0")")/dependencies

sudo apt-get install -y libgtest-dev

# Comes pre-built in Ubuntu 20.04
if [ "$(lsb_release -sr)" = "18.04" ]; then
    cd /usr/src/gtest
    sudo cmake CMakeLists.txt
    sudo make
    sudo cp *.a /usr/lib
fi

cd ${DEP_LOC}
./build_install_gp.sh || exit 1

cd ${DEP_LOC}
./build_install_torch.sh || exit 1