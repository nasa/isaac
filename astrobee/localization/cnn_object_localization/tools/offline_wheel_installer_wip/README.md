WARNING: This tool doesn't work for installing most things to Astrobee, since it can't deal with architecture differences.

### Workflow for installing python wheels on the robots

1. Ensure that `./requirements/wheel_requirements.txt` contains a complete list of end-goal pip packages to install.
2. Run `./scripts/install_wheels`.
3. You'll need to update your `PYTHONPATH` on Astrobee in all new terminals so that Python knows about the installed packages.

### Workflow for uninstalling python wheels on the robots

1. Simply delete the tool folder on Astrobee.