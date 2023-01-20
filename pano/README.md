\page Panorama stitcher

## Overview

This document describes how to stitch panoramas collected by an Astrobee robot following an ISAAC panorama-style survey.

We follow an approach using a Docker container that should be highly repeatable. Much of this process can also be run natively in your host OS if needed, but we don't cover that.

## Install the Docker container for panorama stitching

### Prerequisites

- Make sure the Docker server is installed in your host operating system, following step 1 of the Docker option in the [`astrobee` repo installation instructions](https://github.com/nasa/astrobee/blob/develop/INSTALL.md).

- Check out the source code from the `isaac` repo and set the `ISAAC_WS` environment variable following the [Instructions on installing and using the ISAAC Software](https://nasa.github.io/isaac/html/md_INSTALL.html)

### Build the container

Run:
```bash
$ISAAC_WS/src/pano/docker/build.sh
```

## Stitch a batch of panoramas

### Input requirements

The expected input data for stitching a single panorama is:
- `bag_path`: An Astrobee telemetry bag file recorded during an ISAAC panorama-style survey. The only SciCam image timestamps in the bag should be from within a single panorama.
- `images_dir`: A folder containing the full-resolution SciCam image JPEG files saved on the HLP at the same time when the bag was recorded. Note that it's typical and no problem if a single image folder contains SciCam images that span multiple panoramas (and multiple bag files). However, the stitching script does assume all SciCam images for any given bag are in the same folder.

Multiple panoramas can be stitched in a single batch job.

### Pre-process the inputs

Here are some typical pre-processing steps you might need before stitching when working with real Astrobee data:

- The documentation on [Using Astrobee Robot Telemetry Logs](https://nasa.github.io/astrobee/html/using_telemetry.html) applies. For example, if using bags that have obsolete message types, you might need to run the `rosbag_fix_all.py` script.
- If you have a bag that includes telemetry from multiple panoramas (or other SciCam image timestamps), you should split it up so that each bag contains one panorama and no other SciCam data.
- You may find it more convenient to work with filtered telemetry bags that contains only the messages required by the panorama stitching. This is particularly useful if you need to transfer the bags to a different host before stitching (minimizing transfer data volume and storage required on the stitching host). It also speeds up stitching slightly. You can generate filtered bags like this:
    ```bash
    rosrun bag_processing scripts/rosbag_topic_filter.py -a /hw/cam_sci/compressed -a /loc/pose in1.bag in2.bag
    # generates: filtered_in1.bag filtered_in2.bag
    ```

### Configure folders for processing

```bash
# choose your own folders in the host OS
export ISAAC_PANO_INPUT="$HOME/pano_input"
export ISAAC_PANO_OUTPUT="$HOME/pano_output"
```

The input folder will be mounted read-only in the Docker container at the mount point `/input`. It must contain the SciCam images and input bags for the panoramas (and no other bags). The panoramas will be detected with a recursive search, so the folder structure underneath the input folder shouldn't matter.

The output folder will be mounted read-write in the Docker container at the mount point `/output`. It should typically be empty at the start of the stitching process. Stitched panoramas and other intermediate files will be written there.

These environment variables will be used by the `run.sh` script in the next section.

Because the folders are located in the host OS, stitched panorama outputs will persist even if the Docker container is stopped.

### Start the container

In your host OS, start the stitching container with the command below:
```bash
$ISAAC_WS/src/pano/docker/run.sh
```

This command should open an interactive terminal session running inside the container. If you exit the `run.sh` terminal session, the container will be stopped. Commands typed into the `run.sh` terminal will be run inside the container.

You may also find it convenient to open additional terminals inside the container, which you can do as follows:
```bash
$ISAAC_WS/src/pano/docker/exec.sh bash
```
You can exit terminal sessions started this way without stopping the container.

In the sections below, commands to run inside the container can be typed into one of these `run.sh` or `exec.sh` terminal sessions inside the container, or you can run them directly from the host like this:
```bash
$ISAAC_WS/src/pano/docker/exec.sh mycmd arg1 arg2 ...
```

### Detect and configure the panoramas

Inside the container, run:
```bash
/src/isaac/src/pano/scripts/config_panos.py
```

This will create the panorama config file `pano_meta.yaml` in the output folder. You should verify the config file looks correct by opening `$ISAAC_PANO_OUTPUT/pano_meta.yaml` in your favorite editor in the host OS.

Below is an example `pano_meta.yaml`:
```yaml
scenes:
  scene000_isaac11_bumble_usl_bay4:
    bag_path: /input/isaac11_bumble/isaac11_bumble_usl_bay4.bag
    images_dir: /input/isaac11_bumble/isaac_sci_cam_image_delayed
    robot: bumble
    activity: isaac11
    module: usl
    bay: 4
    extra_stitch_args: ''
  scene001_isaac11_queen_usl_bay1:
    bag_path: /input/isaac11_queen/isaac11_queen_usl_bay1.bag
    images_dir: /input/isaac11_queen/isaac_sci_cam_image_delayed
    robot: queen
    activity: isaac11
    module: usl
    bay: 1
    extra_stitch_args: ''
```

Ideally, the `config_panos.py` script will set all the config fields correctly, but it is not especially smart and could get fooled in some situations. It fills many of the later fields by attempting to parse the `bag_path`. You can help it by renaming your bags to filenames that follow the conventions in the example. If it fails to auto-configure a field, its value will be set to `null`.

Here's what to check in the config file:
- Each entry in the `scenes` list defines a single panorama to stitch. All of your panoramas should appear in the list.
- The header line for each panorama defines its `scene_id`. This id is normally not important, but it does control the name of the output subfolder for that panorama, as well as its scene id in the Pannellum tour. If you find a meaningful id helpful for debugging, you can set it to whatever you like, as long as every panorama has a unique id.
- The `bag_path` and `images_dir` fields should contain valid paths that specify the location of the input data for that panorama. They should match, in that the SciCam image timestamps in the bag should correspond to SciCam images in the folder.
- The `robot` field should correctly specify the robot that collected the panorama. This field is used to look up the correct SciCam lens calibration parameters to use during stitching.
- The `activity`, `module`, and `bay` fields don't affect the stitching step but should be filled in correctly because they are displayed to users in the final output tour.
- The `extra_stitch_args` field provides a way for advanced users to modify the stitching parameters on a per-panorama basis. It would typically be used for debugging and tuning when the stitching process fails.

### Stitch the panoramas

Inside the container, run:
```bash
snakemake -s /src/isaac/src/pano/pano_stitch/scripts/Snakefile -d /output -c1
```

This will trigger the `snakemake` build system to stitch the panoramas. There is one job per panorama in the config file, and `snakemake` will try to run these jobs in parallel up to the number of cores specified in the `-c` argument. (When `-c` is specified with no arguments, it will use the number of cores allocated to the container.)

Note that some of the individual steps within each panorama stitch (e.g., `enblend`) run multi-threaded, and each individually tries to use all available cores, which could cause problems when stitching multiple panoramas in parallel. Both `snakemake` and `enblend` provide ways to manage this, which could be an area for future work.

### Generate the tour

TODO

### View the tour

TODO