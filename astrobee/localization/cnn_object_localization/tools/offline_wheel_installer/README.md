## Overview:

This tool is intended to take a list of requirements, figure out their dependencies, identify which packages need to be installed on Astrobee, download the packages, and install them.

NOTE: The original idea behind the `offline_debian_installer` and `offline_wheel_installer` tools was to use the debian installer to install pip and its prerequisites on Astrobee, and then use the wheel installer to install the necessary Python prerequisites for running the Python handrail segmentation ROS nodes. Both tools function as intended, but unfortunately depend on the availability of pre-built debians and wheels for the armv7l architecture. Several of the Python packages required by the Python handrail segmentation ROS nodes do not support armv7l, and therefore this idea didn't end up working out. Therefore, these tools are provided solely in case they end up useful in the future.

## Workflow for installing python wheels on the robots

1. Ensure that `./requirements/wheel_requirements.txt` contains a complete list of end-goal pip packages to install.
2. Ensure that the platform compatibility tags specified at the top of `./scripts/install_wheels` are correct.
3. Run `./scripts/install_wheels`.
4. You'll need to update your `PYTHONPATH` on Astrobee in all new terminals so that Python knows about the installed packages.

## Workflow for uninstalling python wheels on the robots

1. Simply delete the tool folder on Astrobee.