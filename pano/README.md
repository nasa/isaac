\page Panorama stitcher

## Overview

This document describes how to stitch panoramas collected by an Astrobee robot following an ISAAC panorama-style survey.

We follow an approach using a Docker container that should be highly repeatable. Much of this process can also be run natively in your host OS if needed, but we don't cover that.

## Install the Docker image for panorama stitching

### Prerequisites

- Make sure the Docker server is installed in your host operating system, following step 1 of the Docker option in the [`astrobee` repo installation instructions](https://github.com/nasa/astrobee/blob/develop/INSTALL.md).

- Check out the source code from the `isaac` repo and set the `ISAAC_WS` environment variable following the [Instructions on installing and using the ISAAC Software](https://nasa.github.io/isaac/html/md_INSTALL.html)

### Build the Docker image

Run:
```bash
$ISAAC_WS/src/pano/docker/build.sh
```

## Stitch a batch of panoramas

### Input requirements

The expected input data for stitching a single panorama is:
- `bag_path`: Points to an Astrobee telemetry bag file recorded during an ISAAC panorama-style survey. The only SciCam image timestamps in the bag should be from within a single panorama.
- `images_dir`: Points to a folder containing the full-resolution SciCam image JPEG files saved on the HLP at the same time when the bag was recorded. Note that it's typical and no problem if a single image folder contains SciCam images that span multiple panoramas (and multiple bag files). However, the stitching script does assume all SciCam images for any given bag are in the same folder.

Multiple panoramas can be stitched in a single batch job.

### Pre-process the inputs

Here are some typical pre-processing steps you might need before stitching when working with real Astrobee data:

- The documentation on [Using Astrobee Robot Telemetry Logs](https://nasa.github.io/astrobee/html/using_telemetry.html) applies. For example, if processing older bags that have obsolete message types, you might need to run the `rosbag_fix_all.py` script.
- If you have a bag that includes telemetry from multiple panoramas (or other SciCam image timestamps), you should split it up so that each bag contains one panorama and no other SciCam data.
- You may find it more convenient to work with filtered telemetry bags that contains only the messages required by the panorama stitching. This is particularly useful if you need to transfer the bags to a different host before stitching (minimizing transfer data volume and storage required on the stitching host). It also speeds up stitching slightly. You can generate filtered bags like this:
    ```bash
    rosrun bag_processing scripts/rosbag_topic_filter.py -a /hw/cam_sci/compressed -a /loc/pose in1.bag in2.bag
    # generates: filtered_in1.bag filtered_in2.bag
    ```

### Configure folders for processing

Run:
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
rosrun pano_stitch scripts/config_panos.py
```

This will create the panorama config file `pano_meta.yaml` in the output folder. You should verify the config file looks correct by opening `$ISAAC_PANO_OUTPUT/pano_meta.yaml` in your favorite editor in the host OS.

Below is an example `pano_meta.yaml`:
```yaml
scenes:
  scene000_isaac10_queen_nod2_bay2:
    bag_path: /input/isaac10_queen/20220617_1554_survey_nod2_bay2_std_panorama.bag
    images_dir: /input/isaac10_queen/isaac_sci_cam_image_delayed
    robot: queen
    activity: isaac10
    module: nod2
    bay: 2
    position:
      x: 10.996580718096382
      y: 0.0018100984828177873
      z: 4.899023194998069
    start_time: '2022-06-17T15:57:23.139000Z'
    end_time: '2022-06-17T16:08:06.880000Z'
    extra_stitch_args: ''
    extra_tour_params: {}
  scene001_isaac11_bumble_usl_bay6:
    bag_path: /input/isaac11_bumble/20220711_1238_survey_usl_bay6_std_panorama_run1.bag
    images_dir: /input/isaac11_bumble/isaac_sci_cam_image_delayed
    robot: bumble
    activity: isaac11
    module: usl
    bay: 6
    position:
      x: -0.3593667375653261
      y: 0.0072030887961762385
      z: 4.885617819225414
    start_time: '2022-07-11T12:40:00.841000Z'
    end_time: '2022-07-11T12:51:01.391000Z'
    extra_stitch_args: ''
    extra_tour_params: {}
```

Ideally, the `config_panos.py` script will set all the config fields correctly, but it is not especially smart and could get fooled in some situations. It fills many of the later fields by attempting to parse the `bag_path`. If you want its auto-configure to work better, you can rename your bags in advance to filenames that follow the conventions in the example. If a field is not detected, its value will be set to `null`.

Here's what to check in the config file:
- Each entry in the `scenes` list defines a single panorama to stitch. All of your panoramas should appear in the list.
- The header line for each panorama defines its `scene_id`. This id is normally not important, but it does control the name of the output subfolder for that panorama, as well as its scene id in the Pannellum tour. If you find a meaningful id helpful for debugging, you can set it to whatever you like, as long as every panorama has a unique id.
- The `bag_path` and `images_dir` fields should contain valid paths that specify the location of the input data for that panorama. They should match, in that all of the SciCam image timestamps in the bag should refer to SciCam images in the folder.
- The `robot` field should correctly specify the robot that collected the panorama. This field is used to look up the correct SciCam lens calibration parameters to use during stitching.
- The `activity`, `module`, `bay`, `position`, `start_time`, and `end_time` fields don't affect the stitching step but should be set correctly so they can be displayed to users in the final output tour.
- The `extra_stitch_args` field provides a way for advanced users to pass extra options to the `stitch_panorama.py` script for a specific panorama. It would typically be used for debugging and tuning when the stitching process fails or produces a low-quality result.
- The `extra_tour_params` field provides a way for advanced users to overwrite the parameters for a specific panorama in the `tour.json` file that configures the Pannellum display interface.

### Stitch the panoramas and generate the tour

Inside the container, run:
```bash
snakemake -s /src/isaac/src/pano/pano_view/scripts/Snakefile -d /output -c1
```

This will trigger the `snakemake` build system to stitch the panoramas. There is one job per panorama in the config file, and `snakemake` will try to run these jobs in parallel up to the number of cores specified in the `-c` argument. (When `-c` is specified with no arguments, it will use the number of cores allocated to the container.)

Note that some of the individual steps within each panorama stitch (e.g., `enblend`) run multi-threaded, and each individually tries to use all available cores, which could cause problems when stitching multiple panoramas in parallel. Both `snakemake` and `enblend` provide ways to manage this, which could be an area for future work.

### View the tour

In the host OS, point a web browser at `http://127.0.0.1:8080/`. The `run.sh` script forwards port 8080 of the host to port 80 of the container, where an Apache server is running to provide a preview of the tour.