# Overview

This tool is intended to test the TorchScript compiled model created using the `pytorch_frcnn_training` tool. Much more importantly, it provides documentation on how to code that depends on `libtorch` and `libtorchvision` can be cross-compiled and installed on Astrobee. The below procedures are not particularly elegant or clean, but they work.

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

- `cd <TORCH_INSTALL_DIR>` where `<TORCH_INSTALL_DIR>` is the directory you want `pytorch` to be downloaded and installed in.
- `git clone -b v1.13.1 --recurse-submodule https://github.com/pytorch/pytorch.git`
- `cd pytorch; mkdir build_libtorch; cd build_libtorch`
- `python3 ../tools/build_libtorch.py`
- NOTE: At this point, you may get some complaints about python dependencies. Install as needed and try again.
- NOTE: By default, the build script will try to use maximum parallelism. If your machine/VM starts struggling and/or you get an error `c++: fatal error: Killed signal terminated program cc1plus`, `export MAX_JOBS=4` or some other small number, and then try again.
- `export CMAKE_PREFIX_PATH=<TORCH_INSTALL_DIR>/pytorch/torch/share/cmake/Torch:${CMAKE_PREFIX_PATH}`. Consider adding this or some equivalent to `~/.bashrc`.

## Step 2: Installing libtorchvision:

- `cd <VISION_DOWNLOAD_DIR>` where `<VISION_DOWNLOAD_DIR>` is the directory you want `vision` to be downloaded, but not installed, in.
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

- `./build/libtorch_mrcnn_test <CNN_OBJECT_LOCALIZATION_RESOURCES_PATH>/checkpoints/handrail_finetune_ckpt_199_torchscript.pt` where `<CNN_OBJECT_LOCALIZATION_RESOURCES_PATH>` is the path to the directory containing your checkpoints folder.

# Cross-compiling this tool and running on Astrobee

## Step 1: Getting ready

- Remove the `./CATKIN_IGNORE` file. (It is there so that people uninterested in this functionality don't have to deal with Torch/TorchVision dependencies.)
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
- `sudo su; ./chroot.sh $ARMHF_CHROOT_DIR` (this will give you a shell inside the platform)

## Step 2: Installing libtorch

- Open a separate shell outside of the platform (this is necessary because git isn't installed in the platform, and we need to clone Torch and TorchVision).
- Outside the platform: `cd ${ARMHF_CHROOT_DIR}/root; git clone -b v1.5.0 --recurse-submodule https://github.com/pytorch/pytorch.git`
- Inside the platform: `cd /root/pytorch; python setup.py build`
    - See [this documentation](https://github.com/pytorch/pytorch/blob/4ff3872a2099993bf7e8c588f7182f3df777205b/docs/libtorch.rst) for more info.


## Step 3: Installing libtorchvision

- Outside the platform: `cd ${ARMHF_CHROOT_DIR}/root; git clone -b v0.6.0 --recurse-submodule https://github.com/pytorch/vision.git`
- Inside the platform:  `export CMAKE_PREFIX_PATH=~/pytorch/torch/share/cmake/Torch`
- Inside the platform:  `cd /root/vision; mkdir build; cd build; cmake ..; make; make install`
    - See [this documentation](https://github.com/pytorch/vision/tree/b68adcf9a9280aef02fc08daed170d74d0892361) for more info.

## Step 4: Cross compiling Astrobee

- This is just the normal Astrobee cross-compile process.
- `cd $ASTROBEE_WS`
- `./src/scripts/configure.sh -a; source ~/.bashrc; catkin build`

## Step 5: Cross compiling ISAAC

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
                --whitelist isaac_astrobee_description isaac_util isaac_msgs inspection cargo isaac_hw_msgs wifi isaac gs_action_helper cnn_object_localization
                ```
		- Line: 325
			- Old: 
                ```
                --whitelist isaac_astrobee_description isaac_util isaac_msgs inspection cargo isaac_hw_msgs wifi isaac gs_action_helper
                ```
			- New: 
                ```
                --whitelist isaac_astrobee_description isaac_util isaac_msgs inspection cargo isaac_hw_msgs wifi isaac gs_action_helper cnn_object_localization
                ```
	- File: `${ARMHF_CHROOT_DIR}/usr/local/share/cmake/TorchVision/TorchVisionTargets.cmake` (file is read-only; you'll need to bypass this)
		- Line: 57
			- Old: 
                ```
                INTERFACE_INCLUDE_DIRECTORIES "/root/vision/"
                ```
			- New: 
                ```
                INTERFACE_INCLUDE_DIRECTORIES "${ARMHF_CHROOT_DIR}/usr/local/include"
                ```
- `cd $ISAAC_WS`
- `./src/scripts/configure.sh -a; source ~/.bashrc; catkin build`

## Step 6: Installing and running on Astrobee

- `./scripts/prepare_shared_libraries --root=$ARMHF_CHROOT_DIR --output=$ISAAC_WS/armhf/opt/isaac/lib --libs=libc10.so,libtorchvision.so,libtorch_cpu.so,libtorch.so`
- Follow the normal Astrobee installation procedure for ISAAC. The test executable should be located in `opt/isaac/bin`.
- `scp` over your model weights.
- On Astrobee: `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac/lib`
- On Astrobee: run the test executable.
