This tool is intended to test the TorchScript compiled model created using the `pytorch_mrcnn_training` tool.
The tool depends on having libtorch 1.5.0 and libtorchvision 0.6.0.
The tool can currently only be built and run on native linux, and cannot be cross-compiled.

# Instructions for installing libtorch:
- See `scripts/setup/dependencies/build_install_torch.sh`.

# Instructions for installing libtorchvision:
1. `cd` into a clean directory for downloading TorchVision
2. `wget https://github.com/pytorch/vision/archive/refs/tags/v0.6.0.zip`
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
4. `./build/libtorch_mrcnn_test $CNN_OBJECT_LOCALIZATION_RESOURCES_PATH/checkpoints/handrail_finetune_ckpt_199_torchscript.pt`