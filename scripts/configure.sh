#!/bin/bash
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
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
#
# Helper to configure the astrobee build in a simple a repeatable way
#
# This script simply invoke cmake to configure the build with some
# reasonable options for either Native Linux (-l) or/and ArmHF (-a)
#
# The build path (-b), install path (-p) and build type (-B) have
# default values that can be overriden with the according flags.
# Build path and install path are created by default under the home
# directory.

# First identify more or less where we are (absolute or relative ok)
callcmd=$0
workdir=`pwd`
confdir=`dirname $callcmd`
scriptname=${0##*/}
rootpath=`cd $confdir/..; pwd`

cpu_type=`uname -m`
os_kernel=`uname -s | tr '[:upper:]' '[:lower:]'`
gcc_major=`gcc -dumpversion | cut -f-2 -d .`
archname="${cpu_type}_${os_kernel}_gcc${gcc_major}"

# prefix for installation path
install_path=""

# configure nothing by default
native_build=0
armhf_build=0

# Define our options
use_ctc=" -DUSE_CTC=off"                                       # Use cross compile toolchain for making ARM binaries
use_static_libs=""                                             # Build using static libraries. Will use lots of drive space.
test_coverage=""                                               # Build the code with code coverage options. Not compatible with USE_CTC.
enable_gprof=""                                                # Enable compling with support for profiling wih gprof (the GNU Profiler)."
enable_google_prof=""                                          # Enable support for profiling wih pprof (the Google Profiler).
enable_integration_testing=" -DENABLE_INTEGRATION_TESTING=on"  # Build the integration tests if tests are active.

# Force cache cleaning by default
clean_cache=1

# Default debug mode is off
debug_mode=0

# Build target
target="install"

# short help
usage_string="$scriptname [-h] [-l <linux build>] [-a <arm build>] [-n <profile name>]\
 [-p install_path] [-w workspace_path] [-B build_type] [-c <force clean cache>]\
 [-C <don't clean cache>] \
 [-k <use ccache if available>] [-K <do not use ccache>]\
 [-t <with integration tests, requires gpu>] [-T <without integration test>]\
 [-g <print debug information only>]"
#[-t make_target]

# options to parse
optstring="hlan:p:w:B:cCkKtTg"

# Print the script usage
print_usage()
{
    echo $usage_string
}

# Print the help message (list all the options)
print_help()
{
    echo "scriptname usage:"
    echo $usage_string
    echo -e "\t-l Generate a Native Linux build"
    echo -e "\t-a Generate a cross-compiled ARM build"
    echo -e "\t-n set profile name"
    echo -e "\t-p install_path specify the installation directory"
    echo -e "\t   default=${HOME}/cmake_install_platform"
    echo -e "\t-w workspace_path specify the workspace directory"
    echo -e "\t   default=${ASTROBEE_WS}"
    echo -e "\t-B build_type   specify build type (Debug|Release|RelWithDebInfo)"
    echo -e "\t-c delete the cmake cache before for every modules: default on"
    echo -e "\t   (necessary when re-running buildall with diffent options)"
    echo -e "\t-C do not automatically delete the cmake cache when configure is run"
    echo -e "\t   (need to be specified to avoid cleaning the cache by default)"
    echo -e "\t-k use ccache if available to speed up compilation (default)"
    echo -e "\t-K do not use ccache and turn on native optimizations"
    echo -e "\t-t build with the integration tests, if tests enabled, requires gpu (default)"
    echo -e "\t-T build without integration tests, if tests enabled"
    #    echo -e "\t-t make_target  define the make build target"
    #    echo -e "\t   default (when ommited) is 'install'"
    echo -e "\t   when -t is specified, the configure processs is skipped!"
    echo -e "\t-g prints some debug information and exit"
    echo
    echo "Warning 1: -p and -b, unlike the other flags that are sticky (because"
    echo "cmake cache them), need to be re-issued at each invocation of the"
    echo "script. Otherwise the default values will be used instead."
    echo "Warning 2: when using both -a and -l simultanously, the options"
    echo "-b and -p are ignored since they would override the platform specific"
    echo "behavior (for example -p will be the same for ArmHF and Linux)."
    echo
}

# Parse the command line arguments
parse_args()
{
    while getopts $optstring opt $@
    do
    case $opt in
        "h") print_help
         exit 0
         ;;
        "?") print_usage
         exit 1
         ;;
        "l") native_build=1
         ;;
        "a") armhf_build=1
         ;;
        "n") profile=$OPTARG
         ;;
        "p") install_path=$OPTARG/
         ;;
        "w") workspace_path=$OPTARG/
         ;;
        "B") build_type=$OPTARG
         ;;
        "c") clean_cache=1
         ;;
        "C") clean_cache=0
         ;;
        "k") use_ccache=" -DUSE_CCACHE=on"
         ;;
        "K") use_ccache=" -DUSE_CCACHE=off"
         ;;
        "t") enable_integration_testing=" -DENABLE_INTEGRATION_TESTING=on"
         ;;
        "T") enable_integration_testing=" -DENABLE_INTEGRATION_TESTING=off"
         ;;
        "g") debug_mode=1
         ;;
        *) print_usage
           exit 1
           ;;
    esac
    done
}

# Return the full canonical path of a file
# Arguments:
#   1 -> path to canonicalize
# Return:
#   0 if the path exist, 1 if the path does not exist
#   prints a string with the canonical path + store it in $canonical_path
canonicalize()
{
    freepath=$1
    os_name=`uname -s`
    case $os_name in
    Linux)
        # just use readlink :-)
        canonical_path=`readlink -f $freepath`
        readl_ret=$?
        echo $canonical_path
        if [ $readl_ret == 1 ] ; then
        return 1
        else
        return 0
        fi
        ;;
    Darwin | SunOS)
        # BSD systems do not support readlink :-(
        if [ -d $freepath ] ; then
        # the argument is a directory
        canonical_path=`cd $freepath && pwd -P`
        else
        if [ -f $freepath ] ; then
            # the argument is a file
            freedir=`dirname $freepath`
            freefile=`basename $freepath`
            if [ -L $freepath ] ; then
            canfile=`cd $freedir && stat -f "%Y" $freefile`
            else
            canfile=$freefile
            fi
            candir=`cd $freedir && pwd -P`
            canonical_path="${candir}/${canfile}"
        else
            # given path does not exsit
            # since readlink does not return any string for this
            # scenario, just lets do the same and return an error
            canonical_path=""
            return 1
        fi
        fi
        echo $canonical_path
        return 0
        ;;
    *)
        # echo platform not supported yet
        echo "/${os_name}/is/not/yet/supported/by/canonicalize"
        return 1
        ;;
    esac
}

# Start the real work here...
parse_args $@

# Define the paths to use
ff_path=`canonicalize ${rootpath}`

DIST=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`
if [ "$DIST" = "xenial" ]; then
    ros_version=kinetic
elif [ "$DIST" = "bionic" ]; then
    ros_version=melodic
elif [ "$DIST" = "focal" ]; then
    ros_version=noetic
fi

if [ $debug_mode == 1 ]; then
    echo "script is called from: $workdir"
    echo "script dir is: $confdir"
    echo "scriptname is: $scriptname"
    echo "absolute script path: $rootpath"
    echo "freeflyer canonical path: ${ff_path}"
    echo "Other Options:" ${extra_opts}
    echo "linux build enabled: ${native_build}"
    echo "armhf build enabled: ${armhf_build}"
    echo
fi

extra_opts+=${use_ccache}${use_ros}${use_dds}${use_static_libs}${test_coverage}${use_drivers}
extra_opts+=${enable_gprof}${enable_google_prof}
extra_opts+=${enable_integration_testing}


if [[ $native_build == 0 && $armhf_build == 0 ]] ; then
    echo "Nothing to configure (use -l or -a)..."
    echo "Use $scriptname -h for the full list of options"
    print_usage
fi

if [[ $native_build == 1 && $armhf_build == 1 ]] ; then
    echo -n "Linux and ArmHF invoked simultanously:"
    echo " dropping any option -p and -b!"
    workspace_path=""
    install_path=""
fi

if [ $native_build == 1 ] ; then
    echo "configuring for native linux..."
    catkin init

    # Add our cmake to paths and bashrc
    grep -qF 'source /opt/ros/'$ros_version'/setup.bash' ~/.bashrc || echo 'source /opt/ros/'$ros_version'/setup.bash' >> ~/.bashrc
    cmake_astrobee_path=`catkin locate -s`/cmake
    grep -qF ${cmake_astrobee_path} ~/.bashrc || {
      echo -e '\n' >> ~/.bashrc
      echo 'if [[ ":$CMAKE_PREFIX_PATH:" != *":'${cmake_astrobee_path}':"* ]]; then CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:+"$CMAKE_PREFIX_PATH:"}'${cmake_astrobee_path}'"; fi' >> ~/.bashrc
    }
    source ~/.bashrc

    shell="$SHELL"
    if [[ ${shell}  == *"zsh"* ]]; then
        echo "Setting .zshrc with environment variables..."
        grep -qF 'source /opt/ros/'$ros_version'/setup.zsh' ~/.zshrc || echo 'source /opt/ros/'$ros_version'/setup.zsh' >> ~/.zshrc
        grep -qF ${cmake_astrobee_path} ~/.zshrc || {
            echo -e '\n' >> ~/.zshrc
            echo 'if [[ ":$CMAKE_PREFIX_PATH:" != *":'${cmake_astrobee_path}':"* ]]; then CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:+"$CMAKE_PREFIX_PATH:"}'${cmake_astrobee_path}'"; fi' >> ~/.zshrc
        }
    fi

    catkin profile add ${profile:-native}
    catkin profile set ${profile:-native}
    catkin config --no-extend \
        --build-space ${workspace_path}build \
        --install-space ${install_path:-${workspace_path}install} \
        --devel-space ${workspace_path}devel \
        --log-space ${workspace_path}log \
        --no-install \
        --cmake-args ${extra_opts}


fi

if [ $armhf_build == 1 ] ; then
    echo "configuring for armhf..."
    catkin init
    armhf_opts="-DCMAKE_TOOLCHAIN_FILE=${ff_path}/scripts/build/isaac_cross.cmake -DARMHF_ROS_DISTRO=${ros_version} -DCATKIN_ENABLE_TESTING=off"
    use_ctc=" -DUSE_CTC=on"
    enable_gazebo=""
    build_loc_rviz_plugins=""
    catkin profile add ${profile:-armhf}
    catkin profile set ${profile:-armhf}
    catkin config --extend $ASTROBEE_WS/armhf/opt/astrobee \
        --build-space ${workspace_path:-armhf/}build \
        --install-space ${install_path:-${workspace_path:-armhf/}}opt/isaac \
        --devel-space ${workspace_path:-armhf/}devel \
        --log-space ${workspace_path:-armhf/}logs \
        --install \
        --buildlist isaac_astrobee_description isaac_util isaac_msgs inspection cargo isaac_hw_msgs wifi isaac gs_action_helper \
        --cmake-args -DARMHF_CHROOT_DIR=$ARMHF_CHROOT_DIR ${armhf_opts} ${use_ctc} ${extra_opts} \
            -DCMAKE_BUILD_TYPE=Release
fi
