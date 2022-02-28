\page docker Docker Install

Install dependencies
---------

Install docker tools: https://docs.docker.com/engine/install/ubuntu/

Install docker compose: https://docs.docker.com/compose/install/

Install nvidia-docker: https://github.com/NVIDIA/nvidia-docker.


Building the docker images
---------

To run the demos, you can use the remote images and skip this section.
If you want to build the docker images locally instead of pulling from the remote repository, run:

    ./build.sh [OPTIONS]
The build script will automatically detect the current Ubuntu OS version and define the docker files variables
`UBUNTU_VERSION`, `ROS_VERSION`, and `PYTHON` accordingly.


Options:

If a specific version is desired, the option --xenial, --bionic,
and --focal is used for ubuntu 16.04, 18.04, and 20.04 docker images, respectively.

If you are building the docker images in a virtual machine, because of graphics card passthrough
restriction, the gazebo simulation will not work properly. Therefore the build script has the option --vm
which will only build the images that are used for a virtual machine deployment (simulation runs natively).

If you don't want to run mast or don't have access to it (not a public repository), the use the option --no-mast.

For further help on available options:

	./build --help

Run the docker containers
---------

To run the docker containers:

    ./run.sh [OPTIONS]

It will automatically detect the current Ubuntu OS version. If a specific version is desired, the option
--xenial, --bionic, and --focal is used for ubuntu 16.04, 18.04, and 20.04 docker images, respectively.

Options:

If a specific version is desired, the option --xenial, --bionic,
and --focal is used for ubuntu 16.04, 18.04, and 20.04 docker images, respectively.
Note this is only valid for the docker containers, native software will run on the host OS.






For further help on available options:

	./run --help





http://localhost:8888/lab?token=isaac

Docker Demos
----------

There are currently 3 demos available to showcase some aspects of the ISAAC functionality

### Trigger anomaly

This demo will trigger a C02 anomaly, the levels of C02 will start to increase. The mast detects the anomaly and sends astrobee to inspect a vent. Astrobee will undock, calculate the optimal inspection pose to observe the target and move towards that pose, replanning if any obstacle is found. When the robot has the vent of interest in sight, it will take a picture and run it through a trained CNN, identifying whether the vent is obstructed, free or inconclusive result. After inspection Astrobee will dock autonomously.

### Trigger geometric mapping

This demo will trigger a geometric mapping inspection event. The geometric mapper collects pictures from several poses and creates a 3d mesh of the ISS.
The robot will undock, follow a trajectory taking pictures at the specified waypoints and dock again. For the geometric mapper, the trajectory followed is defined in astrobee/behaviors/inspection/resources/jpm_sliced.txt. The geometric mapper will map a section of the jem containing the entry node.

### Trigger wifi mapping

This demo will trigger a volumetric mapping inspection event. The volumetric mapper collects information from an onboard sensor of Astrobee and interpolates the data in a specified area.
The robot will undock, follow a trajectory and dock again. For the wifi mapper, the trajectory followed is defined in astrobee/behaviors/inspection/resources/wifi.txt.


Open `http://127.0.0.1:8080` in a web browser to see what is happening. Use
`docker ps` to see the docker containers and use `docker exec -it container_name /bin/bash` to get a shell in one.

Cancel with Ctrl+c and then run `scripts/docker/shutdown.sh` to stop the demo.
