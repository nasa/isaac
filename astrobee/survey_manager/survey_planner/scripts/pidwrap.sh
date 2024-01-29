#!/bin/bash

# Copyright (c) 2023, United States Government, as represented by the
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

######################################################################
# Usage: pidwrap.sh <command> [arg1] [arg2] ...

# A wrapper that runs the specified child command with the following added features:
# - The PID of the child is written to ${script_dir}/pid.txt when the child starts.
# - The return code of the child is written to ${script_dir}/return_code.txt when the child exits.

# The ${script_dir} variable refers to the folder containing this script. The reason we write the
# status files to ${script_dir} is that we're assuming a controller script will create a unique
# temp folder each time it wants to wrap a command, will copy this script to that temp folder, then
# run the script like so:
#   /tmp/uniq6537/pidwrap.sh cmd arg1 arg2 ...

# With that context, it's logical for the script to write the status files to the same unique folder
# so that the controller will know where to find them. This also keeps the wrapped command string
# (which will be visible to the operator) relatively short.

# Note: As an external process that doesn't own the child but can read its PID from the PID file,
# here are some ways you could check the child status:
# - Poll whether child is active: Check if directory '/proc/$pid` exists
# - Wait until child finishes: Run 'tail --pid=$pid -f /dev/null'. This exits when the child exits
#   (or immediately if the child already exited).
# - Get more status for active child: Run 'cat /proc/$pid/status'.

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Enable bash job control (needed for fg command). It's normally disabled for non-interactive
# scripts.
set -m

# Run child in background so we can immediately log its PID. We prefix the command with the short
# sleep to avoid a weird edge case for commands that run very quickly, like if you use 'true' and
# 'false' as test commands -- it appears that bash isn't fast enough to put them in the background
# and foreground before they exit, which breaks the logic in the rest of the script.
echo -n "pidwrap: running "
(sleep 0.05 && exec "$@") &
pid=$!
echo $pid > "${script_dir}/pid.txt"

# Foregrounding the child here has two effects: (1) It gives full control of the interactive
# terminal back to the child, which is key because the child might be a command shell. (2) It makes
# the child the current job again such that when it exits, $? is set to its return code.
fg
return_code=$?
echo $return_code > "${script_dir}/return_code.txt"
echo "pidwrap: return code $return_code"

exit $return_code
