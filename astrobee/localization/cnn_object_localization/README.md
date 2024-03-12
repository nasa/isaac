# NOTE:

NOTE: The code in this folder is in a fairly complete state. 
Everything "works" and is reasonably well-documented. 
A minimal C++ test of the model can be cross-compiled and installed to Astrobee.
The only problem is that the Mask-RCNN model used is too large, and can't be loaded into Astrobee's memory.
If you were to hypothetically slap another stick of RAM into Astrobee, the model would probably be able to run.

See the `lightweight_cnn_object_localization` folder for a newer approach to this project. Some key differences include:

- Using a keypoint-based pipeline instead of a mask-based pipeline
- Using a model with MobileNetV3 instead of ResNet-50 as the backbone (orders of magnitude faster/smaller)
- Torch 1.13.1 instead of Torch 1.5.0
- Substantially less complete
    - Essentially just a proof-of-concept that the model works and can run on the robot
    - No integration of the model, either in simulation or on the robot hardware, for actual localization

If you need to build this package, you probably want to remove the `CATKIN_IGNORE` file.

# Overview

This module contains the code for handrail pose estimation. 
The below documentation is in regards to running the handrail pose estimation nodes themselves (relevant code is in `launch`, `msg`, `nodes` and `src`.)
The `tools` folder contains code for generating synthetic training data, as well as for training/testing the model. See the `README.md` files in that folder.
Training data and pre-trained weights are not included in this git repository.

# Running handrail ICP localization nodes in simulation

## Preparation

1. Set `dock_cam_rate = 1.0` in `astrobee/config/simulation/simulation.config`.
2. Check that color dock camera data is being published. If necessary, modify the `image_topic` param value in `./launch/sim_handrail.launch` to point to this topic.
3. Check that hardcoded camera parameters in `./src/cnn_object_localization/mrcnn_utils/undistorter.py` match those in `description/description/urdf/sensor_dock_cam.urdf.xacro`.
4. Install prerequisites in `./requirements.txt`. (Do not use a venv; this will mess with ROS dependencies.)
5. Ensure the reference pointcloud for a 30-inch handrail is available at `$CNN_OBJECT_LOCALIZATION_RESOURCES_PATH/reference_pointclouds/handrail_30.pcd`.
6. Ensure the fine-tuned Mask-RCNN checkpoint is available at `$CNN_OBJECT_LOCALIZATION_RESOURCES_PATH/checkpoints/handrail_finetune_ckpt_199.pth`.

## Running
1. `source` the `devel/setup.bash` files for both Astrobee and ISSAC.
2. Run `export CNN_OBJECT_LOCALIZATION_RESOURCES_PATH=...`
3. Run `roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true pose:="11.2573 -8.200 4.88988 0.00121545 -0.704632 0.00749165 0.709532"` (This launches the Classic Gazebo simulation with Astrobee initialized in a pose such that it is looking at a handrail.) 
4. In another terminal, run `roslaunch ./launch/sim_handrail.launch`. (This launches the three nodes required for handrail segmentation, masking, and ICP.)

# Running handrail ICP localization nodes on the Astrobee

Not currently possible. The code currently has Python ROS nodes that can be installed on Astrobee, but the necessary Python dependencies cannot be installed. The segmentation model can be compiled to TorchScript, and there is a minimal pure C++ test for this model that can be run on Astrobee; however, the full handrail localization functionality of the Python ROS nodes has not been ported to Astrobee yet.

