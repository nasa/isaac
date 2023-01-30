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

PACKAGE_NAME=libtorch

if [ -d $PACKAGE_NAME ]; then
  rm -rf $PACKAGE_NAME
fi
wget --quiet https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.5.0%2Bcpu.zip
sudo unzip -q libtorch-cxx11-abi-shared-with-deps-1.5.0+cpu.zip -d /usr/include
echo 'export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/include/libtorch/share/cmake/Torch' >> ~/.bashrc 
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/include/libtorch/share/cmake/Torch

