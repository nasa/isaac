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
FROM ${REMOTE}/isaac:latest-ubuntu20.04

RUN apt-get update \
  && apt-get install -y \
        python3-pip \
        ffmpeg libsm6 libxext6 \
        ros-noetic-cv-bridge \
  && rm -rf /var/lib/apt/lists/*

RUN pip3 install pyArango \
    && pip3 install jupyterlab jupyterhub nbconvert Pygments==2.6.1 jupyros \
    && pip3 install networkx==3.1 \
    && pip3 install matplotlib opencv-python numpy-quaternion pandas scikit-image \
    && pip3 install torch torchvision --extra-index-url https://download.pytorch.org/whl/cpu \
    && pip3 install scikit-image jellyfish lmdb numpy==1.24.4 \
    && pip3 install ipympl gdown timm nltk pytorch_lightning==1.6.5 \
    && pip3 install --upgrade ipywidgets

EXPOSE 8888

# Setup work directory and add the entrypoint for docker
RUN mkdir /home/analyst \
    && echo "\ncd /home/analyst\njupyter lab --allow-root --no-browser --ip 0.0.0.0\n" >> /ros_entrypoint.sh \
    && cat /ros_entrypoint.sh


# Configure container startup
#ENTRYPOINT ["/usr/bin/bash", "/ros_entrypoint.sh"]
#CMD ["ros_entrypoint.sh"]
