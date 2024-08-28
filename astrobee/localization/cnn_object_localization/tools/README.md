`synthetic_segmentation_data` contains code to generate synthetic training data for the handrail segmentation Mask-RCNN model using Ignition Gazebo.

`pytorch_mrcnn_training` contains code to train, fine-tune, and test the Mask-RCNN model within Python, as well as to convert it to a TorchScript model.

`libtorch_mrcnn_test` contains code to test the TorchScript model within a C++ environment, as well as instructions on how to get `libtorch` and `libtorchvision` to work in a cross-compilation.

`offline_debian_installer` and `offline_wheel_installer` are not useful for this project, and are provided solely in case they end up useful for some other project.