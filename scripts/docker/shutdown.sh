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

thisdir=$(dirname "$(readlink -f "$0")")

# stop isaac container if it exists
docker stop isaac 2>/dev/null
docker rm isaac 2>/dev/null

# stop all docker-compose combinations
docker-compose -f ${thisdir}/docker_compose/ros.docker-compose.yml -f ${thisdir}/docker_compose/idi.docker-compose.yml -f ${thisdir}/docker_compose/mast.docker_compose.yml -f ${thisdir}/docker_compose/analyst.docker-compose.yml down \
||docker-compose -f ${thisdir}/docker_compose/ros.docker-compose.yml -f ${thisdir}/docker_compose/idi.docker-compose.yml -f ${thisdir}/docker_compose/mast.docker_compose.yml down \
|| docker-compose -f ${thisdir}/docker_compose/ros.docker-compose.yml -f ${thisdir}/docker_compose/idi.docker-compose.yml down
