\page cargo Cargo Behavior

This directory provides the cargo_tool

### Using the cargo tool

This tool is used to initiate pickup and drop cargo actions.

To run the tool:
	
	rosrun cargo cargo_tool -$ACTION [OPTIONS]

The actions that can be defined are:
pick - picks up the cargo
drop - drops the cargo

The options that can be defined are:
cargo_pose     - a pose defined in quaternions of the cargo bag

