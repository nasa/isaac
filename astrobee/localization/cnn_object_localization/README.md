This module contains the code for handrail pose estimation.

### Running handrail ICP localization nodes in simulation

1. Set `dock_cam_rate = 1.0` in `astrobee/config/simulation/simulation.config`.
2. Ensure prerequisites in `./requirements.txt` are installed. (Do not use a venv; this will mess with ROS dependencies.)
3. Ensure the reference pointcloud for a 30-inch handrail is available at `$CNN_OBJECT_LOCALIZATION_RESOURCES_PATH/reference_pointclouds/handrail_30.pcd`.
4. Ensure the fine-tuned Mask-RCNN checkpoint is available at `$CNN_OBJECT_LOCALIZATION_RESOURCES_PATH/checkpoints/handrail_finetune_ckpt_199.pth`.
5. Run `export CNN_OBJECT_LOCALIZATION_RESOURCES_PATH=...`
6. `source` the `devel/setup.bash` files for both Astrobee and ISSAC.
7. Run `roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true pose:="11.2573 -8.200 4.88988 0.00121545 -0.704632 0.00749165 0.709532"` (This launches the Classic Gazebo simulation with Astrobee initialized in a pose such that it is looking at a handrail.) 
8. In another terminal, run `roslaunch ./launch/sim_handrail.launch`. (This launches the three nodes required for handrail segmentation, masking, and ICP.)

### Running handrail ICP localization nodes on the Astrobee

Not currently possible.