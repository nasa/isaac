## Overview

This tool is intended to take a list of requirements, figure out their dependencies,identify which packages need to be installed on Astrobee, download the packages, and install them. Critically, it does this while logging it's own actions, and provides a script to reverse the installation.

NOTE: The original idea behind the `offline_debian_installer` and `offline_wheel_installer` tools was to use the debian installer to install pip and its prerequisites on Astrobee, and then use the wheel installer to install the necessary Python prerequisites for running the Python handrail segmentation ROS nodes. Both tools function as intended, but unfortunately depend on the availability of pre-built debians and wheels for the armv7l architecture. Several of the Python packages required by the Python handrail segmentation ROS nodes do not support armv7l, and therefore this idea didn't end up working out. Therefore, these tools are provided solely in case they end up useful in the future.

## Workflow for installing debians on the robots

1. Ensure that `./requirements/debian_requirements.txt` contains a complete list of end-goal debian packages to install. 
2. Run `./scripts/prepare_debians` on the local system.
3. Run `./scripts/install_debians` on Astrobee.

## Workflow for uninstalling debians on the robots

1. On Astrobee, run `./scripts/uninstall_debians` to uninstall debians. (NOTE: The file `temp/debian_dependencies_list_filtered.txt` is necessary to do this. If you accidently deleted it, check the logs folder for a backup.)
2. Remove the tool folder on Astrobee to complete the uninstall.
