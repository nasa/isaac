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

## Torch CMAKE_PREFIX_PATH
#  Astrobee always augments the CMAKE_PREFIX_PATH in .bashrc before testing
#  if zsh is in use. We'll just add a conditional extension to every shell
#  rc file we find (currently only looking at .bashrc and .zshrc)
cmake_isaac_torch_path=/usr/include/libtorch/share/cmake/Torch
for shell_cfg in "~/.bashrc" "~/.zshrc"; do
    if [[ -f ${shell_cfg} ]] && [ $(grep -cF ${cmake_isaac_torch_path} ${shell_cfg}) -eq 0 ]; then
        echo -e '\n## ISAAC Dependency - Torch CMAKE Path\n' >> ${shell_cfg}
        echo 'if [[ ":$CMAKE_PREFIX_PATH:" != *":'${cmake_isaac_torch_path}':"* ]]; then CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:+"$CMAKE_PREFIX_PATH:"}'${cmake_isaac_torch_path}'"; fi' >> ${shell_cfg}
    fi
done
echo "Torch added to CMAKE_PREFIX_PATH in shell config file, source ~/.$(basename ${SHELL})rc before building"
