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
FROM ${REMOTE}/isaac:msgs-ubuntu16.04

RUN apt-get update && apt-get install -y \
  unzip \
  libc6-dev-i386 \
  lib32z1 \
  python-wstool \
  openjdk-8-jdk \
  ros-kinetic-rosjava \
  && rm -rf /var/lib/apt/lists/*

# Install Android
RUN mkdir $HOME/android-sdk \
  && cd $HOME/android-sdk \
  && wget https://dl.google.com/android/repository/tools_r25.2.3-linux.zip \
  && unzip tools_r25.2.3-linux.zip \
  && tools/bin/sdkmanager --update \
  && yes | tools/bin/sdkmanager "platforms;android-25" "build-tools;25.0.2" "extras;google;m2repository" "extras;android;m2repository"

# Compile msg jar files, genjava_message_artifacts only works with bash
RUN ["/bin/bash", "-c", "cd /src/msgs \
  && catkin config \
  && catkin build \
  && . devel/setup.bash \
  && genjava_message_artifacts --verbose -p ff_msgs ff_hw_msgs isaac_msgs isaac_hw_msgs"]

# Copy over the apk source code
COPY apks /src/msgs/src/

# Copy msgs .jar files and build apk
RUN find /src/msgs/devel/share/maven -name *.jar | xargs cp -t /src/msgs/src//isaac_gs_ros_bridge/app/libs \
  && cd /src/msgs/src/isaac_gs_ros_bridge \
  && ANDROID_HOME=$HOME/android-sdk ./gradlew build
