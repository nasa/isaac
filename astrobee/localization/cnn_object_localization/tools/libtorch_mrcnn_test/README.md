This tool is intended to test the TorchScript compiled model created using the `pytorch_mrcnn_training` tool.
The test is intended to be as standalone as possible, isolated from ROS and the rest of Astrobee/ISSAC code.

# Instructions:
1. `wget https://download.pytorch.org/libtorch/nightly/cpu/libtorch-shared-with-deps-latest.zip`
2. `unzip libtorch-shared-with-deps-latest.zip`
3. `rm libtorch-shared-with-deps-latest.zip`
4. `cmake -DCMAKE_PREFIX_PATH=libtorch -S . -B build`
5. `cmake --build build --config Release`
6. `./build/libtorch_mrcnn_test /path/to/torchscript/model.pt`