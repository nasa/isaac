`synthetic_segmentation_data` contains code to generate synthetic training data for the handrail segmentation Mask-RCNN model using Ignition Gazebo.

`pytorch_frcnn_training` contains code to train and test the Faster-RCNN model within Python, as well as to convert it to a TorchScript model.

`libtorch_frcnn_test` contains code to test the TorchScript model within a C++ environment, as well as instructions on how to get `libtorch` and `libtorchvision` to work in a cross-compilation.