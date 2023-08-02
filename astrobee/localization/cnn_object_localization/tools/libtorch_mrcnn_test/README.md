
# Overview

This tool is intended to test the TorchScript compiled model created using the `pytorch_mrcnn_training` tool.

# Building and running this tool locally

## Step 1: Installing libtorch

- See `scripts/setup/dependencies/build_install_torch.sh` for an outline of the procedure. Remember that we want Torch v1.5.0.
- `export CMAKE_PREFIX_PATH=<TORCH_DIR>:${CMAKE_PREFIX_PATH}`, where `<TORCH_DIR>` is the directory in which `TorchConfig.cmake` was installed. (Consider adding this or some equivalent to `~/.bashrc`.)

## Step 2: Installing libtorchvision:

- `cd` into a clean directory for downloading TorchVision v0.6.0.
- `wget https://github.com/pytorch/vision/archive/refs/tags/v0.6.0.zip`
- `unzip v0.6.0.zip; rm v0.6.0.zip`
- `cd vision-0.6.0; mkdir build; cd build`
- `cmake ..; make; make install`
- `export CMAKE_PREFIX_PATH=<VISION_DIR>:${CMAKE_PREFIX_PATH}`, where `<VISION_DIR>` is the directory in which `TorchVisionConfig.cmake` was installed. (Consider adding this or some equivalent to `~/.bashrc`.)

## Step 3: Building the test

- `cmake -S . -B build`
- `cmake --build build --config Release`

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

- Follow the normal Astrobee installation procedure.
- The test executable should be located in `opt/astrobee/bin`.
