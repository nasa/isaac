WARNING: This tool only works if all packages and their dependencies provide binaries for the Astrobee platform.
Pay close attention while downloading packages to ensure none are skipped due to lack of support.

### Workflow for installing python wheels on the robots

1. Ensure that `./requirements/wheel_requirements.txt` contains a complete list of end-goal pip packages to install.
2. Ensure that the platform compatibility tags specified at the top of `./scripts/install_wheels` are correct.
3. Run `./scripts/install_wheels`.
4. You'll need to update your `PYTHONPATH` on Astrobee in all new terminals so that Python knows about the installed packages.

### Workflow for uninstalling python wheels on the robots

1. Simply delete the tool folder on Astrobee.