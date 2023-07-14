
### Workflow for installing debians on the robots

1. Ensure that `./requirements/debian_requirements.txt` contains a complete list of end-goal debian packages to install. 
2. Run `./scripts/prepare_debians` on the local system.
3. Run `./scripts/install_debians` on Astrobee.

### Workflow for uninstalling debians on the robots

1. On Astrobee, run `./scripts/uninstall_debians` to uninstall debians. (NOTE: The file `temp/debian_dependencies_list_filtered.txt` is necessary to do this. If you accidently deleted it, check the logs folder for a backup.)
2. Remove the tool folder on Astrobee to complete the uninstall.
