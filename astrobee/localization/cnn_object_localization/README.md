This module contains the code for handrail pose estimation.

### Running handrail ICP localization nodes in simulation

1. Ensure prerequisites in `./requirements.txt` are installed. (Do not use a venv; this will mess with ROS dependencies.)
2. Ensure the reference pointcloud for a 30-inch handrail is available at `$CNN_OBJECT_LOCALIZATION_RESOURCES_PATH/reference_pointclouds/handrail_30.pcd`.
3. Ensure the fine-tuned Mask-RCNN checkpoint is available at `$CNN_OBJECT_LOCALIZATION_RESOURCES_PATH/checkpoints/handrail_finetune_ckpt_199.pth`.
4. Run `export CNN_OBJECT_LOCALIZATION_RESOURCES_PATH=...`
5. Run `roslaunch ./launch/sim_handrail.launch`. (This launches the three nodes required for handrail segmentation, masking, and ICP.)
6. In another terminal, run `roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true pose:="11.2573 -8.200 4.88988 0.00121545 -0.704632 0.00749165 0.709532"` (This launches the Classic Gazebo simulation with Astrobee initialized in a pose such that it is looking at a handrail.) 

### Running handrail ICP localization nodes on the Astrobee

(TODO)