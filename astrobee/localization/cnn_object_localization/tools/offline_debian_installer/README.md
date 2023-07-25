NOTE: The original idea behind the `offline_debian_installer` tool was to use it to install pip and its prerequisites on Astrobee, and then use `offline_wheel_installer` to install the necessary Python prerequisites for running the Python handrail segmentation ROS nodes. `offline_debian_installer` is able to do its job, but unfortunately `offline_wheel_installer` isn't, since it depends on the availability of pre-built wheels for the armv7l architecture and these wheels aren't available from PyPi for the packages we need, making both `offline_debian_installer` and `offline_wheel_installer` kind of useless in the context of this project.

### Workflow for installing debians on the robots

1. Ensure that `./requirements/debian_requirements.txt` contains a complete list of end-goal debian packages to install. 
2. Run `./scripts/prepare_debians` on the local system.
3. Run `./scripts/install_debians` on Astrobee.

### Workflow for uninstalling debians on the robots

1. On Astrobee, run `./scripts/uninstall_debians` to uninstall debians. (NOTE: The file `temp/debian_dependencies_list_filtered.txt` is necessary to do this. If you accidently deleted it, check the logs folder for a backup.)
2. Remove the tool folder on Astrobee to complete the uninstall.
