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

echo "--------------------------------------------------------------------------------------------------"
echo "ISAAC Data Interface - Status Check"
echo "--------------------------------------------------------------------------------------------------"

# Find out isaac root dir
thisdir=$(dirname "$(readlink -f "$0")")
rootdir=${thisdir}/../..
idi_source=${idi_source:-${rootdir}/../../isaac_data_interface}
export IDI_PATH=${idi_source}

if [ $(docker-compose -f ./docker_compose/ros.docker-compose.yml -f ./docker_compose/idi.docker-compose.yml ps -q | wc -m) -lt 1 ]; then
    echo "ERROR!"
    echo "The IDI doesn't seem to be running."
    echo "You can run the IDI using the following script:"
    echo "./run.sh"
    exit 1
fi

echo "--------------------------------------------------------------------------------------------------"
echo "Docker Compose status:"
echo "--------------------------------------------------------------------------------------------------"
docker-compose -f ./docker_compose/ros.docker-compose.yml -f ./docker_compose/idi.docker-compose.yml ps

echo "--------------------------------------------------------------------------------------------------"
echo "Astrobee Simulation Status:"
echo "--------------------------------------------------------------------------------------------------"
docker container ls | grep isaac

echo "--------------------------------------------------------------------------------------------------"
echo "Running rostopic list inside Astrobee simulation:"
echo "--------------------------------------------------------------------------------------------------"
docker exec -it isaac /ros_entrypoint.sh rostopic list | head

echo "--------------------------------------------------------------------------------------------------"
echo "Running rosnode list inside Astrobee simulation:"
echo "--------------------------------------------------------------------------------------------------"
docker exec -it isaac /ros_entrypoint.sh rosnode list | head

echo "--------------------------------------------------------------------------------------------------"
echo "Showing the tail of the Astrobee simulation logs"
echo "--------------------------------------------------------------------------------------------------"
docker logs isaac | tail

echo "--------------------------------------------------------------------------------------------------"
echo "Showing the tail of ROS bridge's logs"
echo "--------------------------------------------------------------------------------------------------"
docker-compose -f ./docker_compose/ros.docker-compose.yml -f ./docker_compose/idi.docker-compose.yml logs rosbridge | tail

echo "--------------------------------------------------------------------------------------------------"
echo "Showing the tail of the IDI frontend's logs"
echo "--------------------------------------------------------------------------------------------------"
docker-compose -f ./docker_compose/ros.docker-compose.yml -f ./docker_compose/idi.docker-compose.yml logs idi_frontend | tail

echo "--------------------------------------------------------------------------------------------------"
echo "Showing the tail of the IDI backend's logs"
echo "--------------------------------------------------------------------------------------------------"
docker-compose -f ./docker_compose/ros.docker-compose.yml -f ./docker_compose/idi.docker-compose.yml logs idi_backend | tail

echo "--------------------------------------------------------------------------------------------------"
echo "Status Check Complete!"
echo "--------------------------------------------------------------------------------------------------"