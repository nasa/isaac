Native Install
=====

Machine setup
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

**The `isaac` repo depends on some `astrobee` packages, therefore, `astrobee` needs to be installed beforehand.**


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

The configure script prepares your build directory for compiling the code. Note
that `configure.sh` is simply a wrapper around CMake that provides an easy way
of turning on and off options. To see which options are supported, simply run
`configure.sh -h`.

    pushd $ASTROBEE_WS
    ./src/scripts/configure.sh -l
    source ~/.bashrc
    popd

The configure script modifies your ``.bashrc`` to source ``setup.bash`` for 
the current ROS distribution and to set CMAKE_PREFIX_PATH. It is suggested
to examine it and see if all changes were made correctly.

If you want to explicitly specify the workspace and/or install directories, use
instead:

    ./scripts/configure.sh -l -p $INSTALL_PATH -w $WORKSPACE_PATH

*Note: If a workspace is specified but not an explicit install distectory,
install location will be $WORKSPACE_PATH/install.*

To build, run `catkin build` in the `$WORKSPACE_PATH`. Note that depending on your host
machine, this might take in the order of tens of minutes to complete the first
time round. Future builds will be faster, as only changes to the code are
rebuilt, and not the entire code base.

    pushd $ASTROBEE_WS
    catkin build
    popd

If you are working in simulation only, then you're all done!
The next steps are only for running ISAAC onboard Astrobee.


Cross-compiling isaac (NASA only)
---------

To cross-compile ISAAC, one must first cross compile the astobee code using the NASA_INSTALL instructions. Note that `ASTROBEE_WS` must be defined!!!


Cross compiling for the robot follows the same process, except the configure
script takes a `-a` flag instead of `-l`.

    pushd $ISAAC_WS
    ./src/scripts/configure.sh -a
    popd

Or with explicit build and install paths:

    ./scripts/configure.sh -a -p $INSTALL_PATH -w $WORKSPACE_PATH

*Warning: `$INSTALL_PATH` and `$WORKSPACE_PATH` used for cross compiling HAVE to be
different than the paths for native build! See above for the default values 
for these.*

 Once the code has been built, it also installs the code to
a singular location. CMake remembers what `$INSTALL_PATH` you specified, and
will copy all products into this directory.

Install the code on the robot (NASA only)
---------

Once the installation has completed, copy the install directory to the robot.
This script assumes that you are connected to the Astrobee network, as it uses
rsync to copy the install directory to `~/armhf` on the two processors. It 
takes the robot name as an argument. Here we use `p4d`.

    pushd $ISAAC_WS
    ./src/scripts/install_to_astrobee.sh $INSTALL_PATH p4d
    popd

Here, p4d is the name of the robot, which may be different in your case.

Build ISAAC debian (NASA only)
---------

To build a debian you must first confirm that cross-compiling is functional. Once it is:

    ./src/scripts/build/build_debian.sh