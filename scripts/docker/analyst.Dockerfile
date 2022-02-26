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

# This will set up an Astrobee docker container using the non-NASA install instructions.
# You must set the docker context to be the repository root directory


ARG REMOTE=isaac
FROM ${REMOTE}/isaac:msgs-ubuntu20.04

RUN apt-get update \
  && apt-get install -y python3-pip \
  && rm -rf /var/lib/apt/lists/*

RUN pip3 install pyArango \
    && pip3 install torch==1.10.2+cpu torchvision==0.11.3+cpu -f https://download.pytorch.org/whl/cpu/torch_stable.html \
    && pip3 install jupyterlab jupyterhub nbconvert Pygments==2.6.1

EXPOSE 8888

# Setup work directory and add the entrypoint for docker
RUN mkdir /home/analyst \
    && echo "\ncd /home/analyst\njupyter lab --allow-root --no-browser --ip 0.0.0.0\n" >> /ros_entrypoint.sh \
    && cat /ros_entrypoint.sh


# Configure container startup
#ENTRYPOINT ["/usr/bin/bash", "/ros_entrypoint.sh"]
#CMD ["ros_entrypoint.sh"]
