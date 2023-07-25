WARNING: This tool depends on the availability of pre-built wheels for the armv7l architecture and these wheels aren't available from PyPi for the packages we need, making it kind of useless in the context of this project. As such, this tool is provided here for the sole purpose of hypothetical future re-use in a different use case.

### Workflow for installing python wheels on the robots

1. Ensure that `./requirements/wheel_requirements.txt` contains a complete list of end-goal pip packages to install.
2. Ensure that the platform compatibility tags specified at the top of `./scripts/install_wheels` are correct.
3. Run `./scripts/install_wheels`.
4. You'll need to update your `PYTHONPATH` on Astrobee in all new terminals so that Python knows about the installed packages.

### Workflow for uninstalling python wheels on the robots

1. Simply delete the tool folder on Astrobee.