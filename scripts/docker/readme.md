\page docker Docker Install

Install dependencies
---------

Install docker tools: https://docs.docker.com/engine/install/ubuntu/

Install nvidia-docker (optional, to use GPU): https://github.com/NVIDIA/nvidia-docker.


Building the docker images
---------

To run the demos, you can use the remote pre-built images hosted on Github and skip this section.
If you want to build the docker images locally instead of pulling from the remote repository, use:

    ./build.sh [OPTIONS]

**Before** running this script, please check the available options and defaults with:

	./build --help

The build script will automatically detect the current Ubuntu OS version and define the docker files variables
`UBUNTU_VERSION`, `ROS_VERSION`, and `PYTHON` accordingly. If a specific version is desired, the options --xenial
and --focal are used for Ubuntu 16.04 and 20.04 docker images, respectively.

If you don't want to run mast or don't have access to it (not a public repository), the use the option --no-mast.


Run the docker containers
---------

To run the docker containers:

    ./run.sh [OPTIONS]

**Before** running this script, please check the available options and defaults with:

	./run --help

Make sure the default paths are correct, if not configure those options. Read through the
different optional modules to understand if it fits your purpose.

It will automatically detect the current Ubuntu OS version. If a specific version is desired, the options
--xenial and --focal are used for Ubuntu 16.04 and 20.04 docker images, respectively.

Once the command is executed the host location of the modules launched will be printed. Open those paths
on your favorite browser.


Shutdown docker containers
---------

To stop all of the containers, use:

	scripts/docker/shutdown.sh


Docker Demos
----------

There are currently 3 demos available to showcase some aspects of the ISAAC functionality.

Open `http://127.0.0.1:8080` in a web browser to see what is happening. Use
`docker ps` to see the docker containers and use `docker exec -it container_name /bin/bash` to get a shell in one.

Cancel with Ctrl+c and then run `scripts/docker/shutdown.sh` to stop the demo.


### Trigger anomaly (with MAST Only)

    ./demos/trigger_anomaly.sh

This demo will trigger a C02 anomaly, the levels of C02 will start to increase. The mast detects the anomaly and sends astrobee to inspect a vent. Astrobee will undock, calculate the optimal inspection pose to observe the target and move towards that pose, replanning if any obstacle is found. When the robot has the vent of interest in sight, it will take a picture and run it through a trained CNN, identifying whether the vent is obstructed, free or inconclusive result. After inspection Astrobee will dock autonomously.

### Trigger geometric mapping

    ./demos/trigger_geometric_mapping.sh

This demo will trigger a geometric mapping inspection event. The geometric mapper collects pictures from several poses and creates a 3d mesh of the ISS.
The robot will undock, follow a trajectory taking pictures at the specified waypoints and dock again. For the geometric mapper, the trajectory followed is defined in astrobee/behaviors/inspection/resources/geometry_iss.txt. The geometric mapper will map a section of the jem containing bay 5.

### Trigger wifi mapping

    ./demos/trigger_wifi_mapping.sh

This demo will trigger a volumetric mapping inspection event. The volumetric mapper collects information from an onboard sensor of Astrobee and interpolates the data in a specified area.
The robot will undock, follow a trajectory and dock again. For the wifi mapper, the trajectory followed is defined in astrobee/behaviors/inspection/resources/volumetric_iss.txt.
