\page cargo Cargo Behavior

This directory provides the cargo_tool

# Using the cargo tool

This tool is used to initiate pickup and drop cargo actions.

To run the tool:
	
	rosrun cargo cargo_tool -$ACTION [OPTIONS]

The actions that can be defined are:
pick - picks up the cargo
drop - drops the cargo

The options that can be defined are:
cargo_pose     - a pose defined in quaternions of the cargo bag

# Demo example

Launch isaac simulation normally

Spawn cargo:

    roslaunch isaac_gazebo spawn_object.launch spawn:=cargo pose:="11.3 -5.6 5.6 -0.707 0 0 0.707" name:=CTB_05_1070

Pick up cargo (make sure astrobee is undocked) - the pose is the cargo pose inserted above:

    rosrun cargo cargo_tool -id CTB_05_1070 -pick -pose "11.3 -5.6 5.6 -0.707 0 0 0.707"

Drop cargo - the pose is the dock pose:

    rosrun cargo cargo_tool -drop -pose "10.4 -5.6 5.855 -0.707 0 0 0.707"