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

FROM ros:kinetic

# try to suppress certain warnings during apt-get calls
ARG DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# install of apt-utils suppresses bogus warnings later
RUN apt-get update && \
    apt-get install -y apt-utils 2>&1 | grep -v "debconf: delaying package configuration, since apt-utils is not installed" && \
    apt-get -y install \
        apt-transport-https \
        autoconf \
        autogen \
        build-essential \
        cmake \
        curl \
        g++-multilib \
        gcc-multilib \
        git \
        lcov \
        lib32z1 \
        libboost-dev \
        libc6 \
        libcurl4-openssl-dev \
        libjsoncpp-dev \
        libncurses5-dev \
        libstdc++6 \
        libtool \
        python-catkin-tools \
        python-empy \
        python-pip \
        python-setuptools \
        python-wheel \
        ros-kinetic-cv-bridge \
        ros-kinetic-image-transport \
        uuid-dev \
        unzip \
        wget \
    && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir build_dir && cd build_dir && \
    curl -OLs https://download.arangodb.com/arangodb36/DEBIAN/Release.key && \
    apt-key add - < Release.key && \
    echo 'deb https://download.arangodb.com/arangodb36/DEBIAN/ /' | tee /etc/apt/sources.list.d/arangodb.list && \
    apt-get update && \
    apt-get install -y \
      apt-transport-https \
      arangodb3=3.6.1-1 \
    && \
    rm -r /build_dir && \
    rm -rf /var/lib/apt/lists/*

#RUN pip install --upgrade pip catkin_pkg rosdep catkin_tools vcstool

## cmake
#RUN mkdir /build_dir && cd /build_dir && \
#    wget --quiet https://cmake.org/files/v3.12/cmake-3.12.2.tar.gz && \
#    tar zxf cmake-3.12.2.tar.gz && \
#    cd cmake-3.12.2 && \
#    ./bootstrap && \
#    make -j`nproc` && \
#    make install && \
#    rm -r /build_dir

# install boost
#RUN mkdir /build_dir && cd /build_dir && \
#    wget --quiet https://dl.bintray.com/boostorg/release/1.71.0/source/boost_1_71_0.tar.gz && \
#    tar -xf boost_1_71_0.tar.gz && \
#    cd boost_1_71_0/ && \
#    ./bootstrap.sh && \
#    ./b2 -j8 && \
#    ./b2 install && rm -r /build_dir

# suppress detached head warnings later
RUN git config --global advice.detachedHead false

# google test
RUN mkdir /build_dir && cd /build_dir && \
    git clone --quiet https://github.com/google/googletest.git 2>&1 && \
    cd googletest && \
    git checkout release-1.8.0 2>&1 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j`nproc` && \
    make install && \
    rm -r /build_dir

# yaml-cpp
RUN mkdir /build_dir && cd /build_dir && \
    git clone --quiet https://github.com/jbeder/yaml-cpp 2>&1 && \
    cd yaml-cpp && \
    git checkout yaml-cpp-0.6.2 2>&1 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_SHARED_LIBS=ON .. && \
    make -j`nproc` && \
    make install && \
    rm -r /build_dir

# catkin
#RUN mkdir /build_dir && cd /build_dir && \
#    git clone https://github.com/ros/catkin.git && \
#    cd catkin && \
#    git checkout kinetic-devel && \
#    mkdir build && \
#    cd build && \
#    cmake .. && \
#    make -j`nproc` && \
#    make install && \
#    rm -r /build_dir

# Eigen
RUN mkdir /build_dir && cd /build_dir && \
    wget --quiet http://bitbucket.org/eigen/eigen/get/3.3.5.tar.gz && \
    tar zxf 3.3.5.tar.gz && \
    cd eigen-eigen-b3f3d4950030 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make install -j`nproc` && \
    rm -r /build_dir

# popf
RUN mkdir /build_dir && cd /build_dir && \
    wget --quiet https://github.com/KCL-Planning/ROSPlan/raw/master/rosplan_planning_system/common/bin/popf && \
    chmod u+x popf && \
    mkdir -p /opt/bin && \
    cp popf /opt/bin/ && \
    rm -r /build_dir

RUN mkdir /build_dir && cd /build_dir && \
    git clone --quiet https://github.com/jpbarrette/curlpp.git 2>&1 && \
    cd curlpp && \
    cmake . && \
    sudo make install && \
    sudo ldconfig && \
    rm -r /build_dir

# grep commands to suppress warnings
RUN rosdep update 2>&1 | egrep -v 'as root|fix-permissions' && \
    mkdir /mast_ws && cd /mast_ws && mkdir src && \
    catkin init && \
    catkin config --install

COPY ./isaac/communications/isaac_msgs/ /mast_ws/src/isaac_msgs
RUN . /opt/ros/kinetic/setup.sh && cd /mast_ws/src/isaac_msgs && catkin build --this

COPY ./mast/arangodb_curl /mast_ws/src/arangodb_curl
RUN . /opt/ros/kinetic/setup.sh && cd /mast_ws/src/arangodb_curl && catkin build --this
COPY ./mast/mast_core /mast_ws/src/mast_core
RUN . /opt/ros/kinetic/setup.sh && cd /mast_ws/src/mast_core && catkin build --this --cmake-args -DMAST_64=True
COPY ./mast/iss_ros_publisher /mast_ws/src/iss_ros_publisher
RUN . /opt/ros/kinetic/setup.sh && cd /mast_ws/src/iss_ros_publisher && catkin build --this
COPY ./mast/mast_iss /mast_ws/src/mast_iss
RUN . /opt/ros/kinetic/setup.sh && cd /mast_ws/src/mast_iss && catkin build --this

COPY ./mast/slm_arango_data/slm_data.tar.gz /src/slm_data.tar.gz
RUN /etc/init.d/arangodb3 start && sleep 2 && cd /src && tar -xzf slm_data.tar.gz && arangorestore --input-directory "dump" --server.password "" && arangosh --server.password "" --javascript.execute-string "require(\"org/arangodb/users\").update(\"root\", \"isaac\");" && /etc/init.d/arangodb3 stop

RUN echo "#!/bin/bash\nset -e\n\nsource \"/opt/ros/\$ROS_DISTRO/setup.bash\"\nsource \"/mast_ws/install/setup.bash\"\nexport LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH\nexec \"\$@\"" > /ros_entrypoint.sh
RUN echo "/etc/init.d/arangodb3 start\n\nsleep 2\nroslaunch iss_ros_publisher iss_telemetry.launch &\nsleep 2\n/mast_ws/install/lib/mast_iss/issMain\n" > /run_mast.sh && chmod +x /run_mast.sh
