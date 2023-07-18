This tool is intended to test the TorchScript compiled model created using the `pytorch_mrcnn_training` tool.
The test is intended to be as standalone as possible, isolated from ROS and the rest of Astrobee/ISSAC code.

# Instructions for installing libtorch:
1. `cd $LIBTORCH_DIR`
2. `wget https://download.pytorch.org/libtorch/nightly/cpu/libtorch-shared-with-deps-latest.zip`
3. `unzip libtorch-shared-with-deps-latest.zip`
4. `rm libtorch-shared-with-deps-latest.zip`
5. `export CMAKE_PREFIX_PATH=$LIBTORCH_DIR:${CMAKE_PREFIX_PATH}` (consider adding to .bashrc)

# Instructions for installing libtorchvision:
1. `cd` into a clean directory for downloading TorchVision
2. `git clone https://github.com/pytorch/vision.git`
3. `mkdir build`
4. `cd build`
5. `cmake ..`
6. `make`
7. `make install`
8. Export `$VISION_DIR` as the directory where `TorchVisionConfig.cmake` was installed
8. `export CMAKE_PREFIX_PATH=$VISION_DIR:${CMAKE_PREFIX_PATH}` (consider adding to .bashrc)

# Instructions for building and running the test:
1. `cd $TEST_DIR`
2. `cmake -S . -B build`
3. `cmake --build build --config Release`
4. `./build/libtorch_mrcnn_test /home/astrobee/large_files/checkpoints/handrail_finetune_ckpt_199_torchscript.pt`