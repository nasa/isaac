# Overview

This tool is intended to test the TorchScript compiled model created using the `pytorch_cnn_training` tool. Much more importantly, it provides documentation on how code that depends on `libtorch` and `libtorchvision` can be cross-compiled and installed on Astrobee. The below procedures are not particularly elegant or clean, but they work.

# Building and running this tool locally

- Make sure you are starting from scratch and don't have residue from previous installations of `libtorch` or `libtorchvision` lying around. This can cause mayhem, especially in the case of `libtorchvision`.
- You will need `sudo` privileges.

## Step 1: Installing libtorch

### Option 1: Use a pre-built package

- `cd <TORCH_INSTALL_DIR>` where `<TORCH_INSTALL_DIR>` is the directory you want `libtorch` to be downloaded and installed in.
- `wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.13.1%2Bcpu.zip`
- `unzip libtorch-cxx11-abi-shared-with-deps-1.13.1+cpu.zip; rm libtorch-cxx11-abi-shared-with-deps-1.13.1+cpu.zip`
- `export CMAKE_PREFIX_PATH=<TORCH_INSTALL_DIR>/libtorch/share/cmake/Torch:${CMAKE_PREFIX_PATH}`. Consider adding this or some equivalent to `~/.bashrc`.

### Option 2: Build from scratch

- `cd <TORCH_INSTALL_DIR>` where `<TORCH_INSTALL_DIR>` is the directory you want `pytorch` to be downloaded, built, and installed in.
- `git clone -b v1.13.1 --recurse-submodule https://github.com/pytorch/pytorch.git`
- `cd pytorch; mkdir build_libtorch; cd build_libtorch`
- `export USE_XNNPACK=0; export USE_CUDA=0; export USE_CUDNN=0; export USE_DISTRIBUTED=0; export USE_MKLDNN=0; export BUILD_TEST=0`.
    - `USE_XNNPACK=0` because a bug in the Astrobee cross-compile toolchain prevents us from building it when cross-compiling, so we're setting it here too for consistency's sake.
    - `USE_CUDA=0` and `USE_CUDNN=0` because the Astrobee doesn't have an NVidia GPU.
    - `USE_DISTRIBUTED=0` because we aren't doing any distributed processing stuff.
    - `USE_MKLDNN=0` because the Astrobee doesn't have an Intel CPU.
    - `BUILD_TEST=0` to save time.
- `python3 ../tools/build_libtorch.py`
    - At this point, the script may complain about not having `typing_extensions`. Simply run `pip3 install typing_extensions` and try again.
    - The script will automatically set the number of parallel jobs to the maximum. If your machine/VM starts struggling and/or you get an error `c++: fatal error: Killed signal terminated program cc1plus`, reduce the number of parallel jobs using `export MAX_JOBS=...` and then try again.
- `export CMAKE_PREFIX_PATH=<TORCH_INSTALL_DIR>/pytorch/torch/share/cmake/Torch:${CMAKE_PREFIX_PATH}`. Consider adding this or some equivalent to `~/.bashrc`.

## Step 2: Installing libtorchvision:

- `cd <VISION_DOWNLOAD_DIR>` where `<VISION_DOWNLOAD_DIR>` is the directory you want `vision` to be downloaded and built, but not installed, in.
- `git clone -b v0.14.1 --recurse-submodule https://github.com/pytorch/vision.git`
- `cd vision; mkdir build; cd build`
- `cmake ..`
- `make`
- `sudo make install`
- `export CMAKE_PREFIX_PATH=<VISION_CONFIG_DIR>:${CMAKE_PREFIX_PATH}`, where `<VISION_CONFIG_DIR>` is the directory in which `TorchVisionConfig.cmake` was installed. This is probably something like `/usr/local/share/cmake/TorchVision`. (Consider adding this or some equivalent to `~/.bashrc`.)

## Step 3: Building the test

- `cd` back into this tool directory.
- `cmake -S src -B build`
- `cd build`
- `make`

## Step 4: Running the test

- `./libtorch_frcnn_test <PATH/TO/TORCHSCRIPT/MODEL>`

# Cross-compiling this tool and running on Astrobee

- Make sure you are starting from scratch and don't have residue from previous installations of `libtorch` or `libtorchvision` lying around in your cross-compilation rootfs. This can cause mayhem, especially in the case of `libtorchvision`.
- You will need `sudo` privileges.
- To keep things simple, I choose to build everything in `${ARMHF_CHROOT_DIR}/root`. You can choose somewhere else.

## Step 1: Getting ready

- Ensure that your Astrobee install is set up for cross-compile.
- Download or copy `chroot.sh` from https://babelfish.arc.nasa.gov/bitbucket/projects/ASTROBEE/repos/astrobee_platform/browse/rootfs/chroot.sh.
- Make the following modifications (this makes something wonky but hopefully the subsequent steps still work):
	- File: `chroot.sh`
		- Lines: 34-38
		    - Old: 
                ```
                chroot "$r" mount -t sysfs sysfs /sys
                add_trap chroot "$r" umount /sys
                
                chroot "$r" mount -t proc proc /proc
                add_trap chroot "$r" umount /proc
                ```
		    - New:
                ```
                # chroot "$r" mount -t sysfs sysfs /sys
                # add_trap chroot "$r" umount /sys
                
                # chroot "$r" mount -t proc proc /proc
                # add_trap chroot "$r" umount /proc
                ```
- `sudo su`
- `./chroot.sh $ARMHF_CHROOT_DIR` (this will give you a shell inside the platform)

## Step 2: Installing libtorch

- Open a separate shell outside of the platform.

### Step 2.1: typing_extensions

- Outside the platform: `cd ${ARMHF_CHROOT_DIR}/root; pip3 install typing_extensions --target typing_extensions`
- Inside the platform: `export PYTHONPATH=/root/typing_extensions:$PYTHONPATH`

### Step 2.2: libtorch

- Outside the platform: `cd ${ARMHF_CHROOT_DIR}/root; git clone -b v1.13.1 --recurse-submodule https://github.com/pytorch/pytorch.git`
- Inside the platform: `cd /root/pytorch; mkdir build_libtorch; cd build_libtorch`
- Inside the platform: `export USE_XNNPACK=0; export USE_CUDA=0; export USE_CUDNN=0; export USE_DISTRIBUTED=0; export USE_MKLDNN=0; export BUILD_TEST=0`
    - `USE_XNNPACK=0` because a bug in the toolchain causes it to fail to build. [This bug is fixed in a more up-to-date version of gcc](https://gcc.gnu.org/bugzilla/show_bug.cgi?id=101723), but I don't want to poke around the toolchain stuff. If the version of gcc used ever gets upgraded, you can rebuild everything with XNNPACK and probably get a reasonable bump in model inference time in exchange for your efforts.
    - `USE_CUDA=0` and `USE_CUDNN=0` because the Astrobee doesn't have an NVidia GPU.
    - `USE_DISTRIBUTED=0` because we aren't doing any distributed processing stuff.
    - `USE_MKLDNN=0` because the Astrobee doesn't have an Intel CPU.
    - `BUILD_TEST=0` to save time.
- Inside the platform: `python3 ../tools/build_libtorch.py`
    - The script will attempt to set the number of parallel jobs to the maximum, which it erroneously believes to be 2. You can probably afford to speed up the build process by increasing the number of parallel jobs using `export MAX_JOBS=...`. If your machine/VM starts struggling and/or you get an error `c++: fatal error: Killed signal terminated program cc1plus`, you were too aggressive; back off and try again.

## Step 3: Installing libtorchvision

- Outside the platform: `cd ${ARMHF_CHROOT_DIR}/root`
- Outside the platform: `git clone -b v0.14.1 --recurse-submodule https://github.com/pytorch/vision.git`
- Inside the platform:  `cd /root/vision; mkdir build; cd build`
- Inside the platform:  `export CMAKE_PREFIX_PATH=~/pytorch/torch/share/cmake/Torch:$CMAKE_PREFIX_PATH`
- Inside the platform: `export USE_XNNPACK=0; export USE_CUDA=0; export USE_CUDNN=0; export USE_DISTRIBUTED=0; export USE_MKLDNN=0; export BUILD_TEST=0`
- Inside the platform: `cmake ..`
- Inside the platform: `make`
- Inside the platform: `make install`

## Step 4: Cross compiling Astrobee

- This is just the normal Astrobee cross-compile process.
- `cd $ASTROBEE_WS`
- `./src/scripts/configure.sh -a; source ~/.bashrc; catkin build`

## Step 5: Cross compiling ISAAC

- Remove the `../../CATKIN_IGNORE` file. (It is there so that people uninterested in this functionality don't have to deal with Torch/TorchVision dependencies.)
- Make the following modifications:
	- File: `${ISAAC_WS}/src/scripts/configure.sh`
		- Line: 306
			- Old: 
                ```
                armhf_opts="-DCMAKE_TOOLCHAIN_FILE=${ff_path}/scripts/build/isaac_cross.cmake -DARMHF_ROS_DISTRO=${ros_version} -DCATKIN_ENABLE_TESTING=off
                ```
			- New: 
                ```
                armhf_opts="-DCMAKE_TOOLCHAIN_FILE=${ff_path}/scripts/build/isaac_cross.cmake -DARMHF_ROS_DISTRO=${ros_version} -DCATKIN_ENABLE_TESTING=off -DEXTRA_ROOT_PATH=${ARMHF_CHROOT_DIR}/root/pytorch/torch/share/cmake/Torch;${ARMHF_CHROOT_DIR}/usr/local/share/cmake/TorchVision"
                ```
		- Line: 322
			- Old: 
                ```
                --whitelist isaac_astrobee_description isaac_util isaac_msgs inspection cargo isaac_hw_msgs wifi isaac gs_action_helper
                ```
			- New: 
                ```
                --whitelist isaac_astrobee_description isaac_util isaac_msgs inspection cargo isaac_hw_msgs wifi isaac gs_action_helper lightweight_cnn_object_localization
                ```
		- Line: 325
			- Old: 
                ```
                --whitelist isaac_astrobee_description isaac_util isaac_msgs inspection cargo isaac_hw_msgs wifi isaac gs_action_helper
                ```
			- New: 
                ```
                --whitelist isaac_astrobee_description isaac_util isaac_msgs inspection cargo isaac_hw_msgs wifi isaac gs_action_helper lightweight_cnn_object_localization
                ```
- `cd $ISAAC_WS`
- `./src/scripts/configure.sh -a; source ~/.bashrc; catkin build`

## Step 6: Installing and running on Astrobee

- `./scripts/prepare_shared_libraries --root=$ARMHF_CHROOT_DIR --output=$ISAAC_WS/armhf/opt/isaac/lib --libs=libc10.so,libtorchvision.so,libtorch_cpu.so,libtorch.so`
    - This script finds the specified `.so` files in the root directory, then copies them to the output directory.
- Follow the normal Astrobee installation procedure for ISAAC. The test executable should be located in `opt/isaac/bin`.
- `scp` over your model weights.
- On Astrobee: `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac/lib`
- On Astrobee: run the test executable.
