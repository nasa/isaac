Native Install
=====

Usage instructions for non-NASA and NASA users
---------

Install the 64-bit version of [Ubuntu 16.04, 18.04 or 20.04](http://releases.ubuntu.com/)
on a host machine, and make sure that you can checkout and build code.

    sudo apt-get install build-essential git

*Note: You will need 4 GBs of RAM to compile the software. If you don't have
that much RAM available, please use swap space.*

*Note: Please ensure you install Ubuntu 16.04, 18.04 or 20.04. At this time we do not support
any other operating system or Ubuntu versions.*

*Note: Please ensure you install the 64-bit version of Ubuntu. We do not
support running ISAAC Software on 32-bit systems.*

Machine setup
---------

The `isaac` repo depends on some `astrobee` packages, therefore, `astrobee` needs to be installed beforehand.

Checkout the project source code
---------

At this point you need to decide where you'd like to put the ISAAC workspace and code
(`ISAAC_WS`) on your machine (add this to your .bashrc for persistency):

    export ISAAC_WS=$HOME/isaac

First, clone the flight software repository:

    git clone --recursive https://github.com/nasa/isaac.git \
    --branch develop $ISAAC_WS/src/

Checkout the submodule:

    git submodule update --init --recursive


Dependencies
---------

Next, install all required dependencies:
*Note: `root` access is necessary to install the packages below*
*Note: Before running this please ensure that your system is completely updated
    by running 'sudo apt-get update' and then 'sudo apt-get upgrade'*

    pushd $ISAAC_WS/src
    cd scripts/setup
    ./install_desktop_packages.sh
    ./build_install_dependencies.sh
    sudo rosdep init
    rosdep update
    popd

Configuring the build
---------

By default, the catkin uses the following paths:
  - devel build path: `$ISAAC_WS/devel`
  - install build path: `$ISAAC_WS/install`

Building the code
---------
    
Source your astrobee build environment, for example as:

    source $ASTROBEE_WS/devel/setup.bash

Build with catkin build.

    cd $ISAAC_WS
    catkin init
    catkin build
    source devel/setup.bash

The command 'source devel/setup.bash' is very important to make sure that everything is correctly linked! Everytime some major change in package is made, a full build should be done as:

    catkin clean
    catkin build
    source devel/setup.bash

If you are working in simulation only, then you're all done!
The next steps are only for running ISAAC onboard Astrobee.

Cross-compiling isaac
---------


To cross-compile ISAAC, one must first cross compile the astobee code using the NASA_INSTALL instructions. Note that ASTROBEE_WS must be defined.

A new catkin profile should be made to retain the configurations and easily switch between normal build.
    
    catkin profile add cross
    catkin profile set cross
    catkin config --extend $ASTROBEE_WS/armhf/opt/astrobee \
                  --build-space armhf/build \
                  --install-space armhf/opt/isaac \
                  --devel-space armhf/devel \
                  --log-space armhf/logs \
                  --whitelist isaac_astrobee_description isaac_util isaac_msgs inspection cargo isaac_hw_msgs wifi isaac gs_action_helper \
                  --install \
                  --cmake-args -DCMAKE_TOOLCHAIN_FILE=$ISAAC_WS/src/scripts/build/isaac_cross.cmake \
                    -DARMHF_CHROOT_DIR=$ARMHF_CHROOT_DIR

Build ISAAC debian
---------

To build a debian you must first confirm that cross-compiling is functional. Once it is:

    ./src/scripts/build/build_debian.sh