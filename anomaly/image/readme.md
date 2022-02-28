\page ano Anomaly Detector

Image Anomaly Detector
====================

Overview
---------

The Image anomaly detector contains a set of tools to analyse incoming images, using Convolutional Neural Networks,  CNN's. To build, train and test the CNN's we use PyTorch.


TorchLib
---------

This package is needed in the anomaly/img_analysis node, such that we can analyse the image, looking for anomalies.
The first step is to download the LibTorch ZIP archive, the link might change, best to go to https://pytorch.org/ and select Linux->LibTorch->C++/Java

Important!: The link is the one labeled '(cxx11 ABI)'. If you select the '(Pre-cxx11 ABI)', it will break ROS:

	wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.5.0%2Bcpu.zip

It is advised to unzip the package into a general directory as '/usr/include'

	unzip libtorch-shared-with-deps-latest.zip

To link the path, add this to your '$HOME/.bashrc' file:

	export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/path/to/libtorch/share/cmake/Torch


Define and train the CNN
---------

The python code containing the CNN definition and training is in resources/vent_cnn.py

Parameters:
data_dir         - path to the dataset. The dataset should have the correct structure for data import. Should be the same as 'path_dataset' in the Get Training data arguments.
classes          - specify the image classes, each class should be a folder name in the test and train folder, default classes is ['free', 'obstacle', 'unknown']. Free meas that it detected a free vent, obstacle means that the vent contains an obstacle, unknown means that the vent was not detected.
num_epochs       - number of epochs to train, default 30
model_name       - saved model name, default "model_cnn.pt"
trace_model_name - saved traced model name, default "traced_model_cnn.pt"

Get training data
---------

To get training data, a tool is available which will read the poses from a vents file and others file. The tool will change the robot's pose and take pictures automatically. For the should be activated when the simulation is spawned like so (should be spawned in an undocked position such that the dock simulation does not interfere with the manual set of the pose):

	roslaunch isaac sim.launch pose:="10.5 -9 5 0 0 0 1"

To run the too:

	rosrun img_analysis get_train_data -path_dataset $PATH_DATASET -vent_poses $VENT_POSES -other_poses $OTHER_POSES [OPTIONS]

Arguments:
path_dataset        - Path to where to save the datasets, mandatory to define. 
vent_poses          - .txt file containing the vent poses
other_poses         - .txt file containing the other non-vent poses
robot_dist          - Robot's distance to vent, standard is 1m
train_pics_per_vent - Number of pictures taken per vent/other for train data
test_pics_per_vent  - Number of pictures taken per vent/other for test data

Test single picture
---------

There is a script, analyse_img.py, in the resources/ folder, which takes as argument the path of a picture taken with the sci_cam, processing it and outputing the classification result. This algorithm is useful to make sure that the C++ API for Pytorch is working properly.

Parameters:
image - path of image to analyse