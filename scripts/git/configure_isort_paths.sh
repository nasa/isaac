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

# Updates the src_paths setting in the .isort.cfg file at the top
# level. See that file for more details.

thisdir=$(dirname "$0")
srcdir=$(cd $thisdir/../.. && pwd)

cd $srcdir

# Generate a comma-separated list of folders containing *.py files
pydirs=$( \
    find . -name "*.py" -print0 \
    | xargs -0 dirname \
    | cut -c3- \
    | grep -Pv "^cmake|^submodules|^astrobee/survey_manager/survey_dependencies|^scripts/setup/dependencies" \
    | sort \
    | uniq \
    | paste -sd "," - \
)

# Overwrite the src_paths line in the config file to use the list
perl -i -ple "if (/^src_paths = /) { \$_ = 'src_paths = $pydirs'; }" .isort.cfg
