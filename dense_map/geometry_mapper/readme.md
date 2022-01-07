\page geometric_streaming_mapper Geometry mapper and streaming mapper

## Overview

This document describes how to process and fuse depth and image data
coming from an Astrobee robot. The two main tools are:

 - The geometry mapper: This tool handles offline processing of robot
   data to create a dense 3D model of robot's environment and a
   texture overlaid on top of it.

 - The streaming mapper: This is a ROS node that takes as input a
    previously computed 3D model using the geometry mapper, a current
    image and pose as the robot flies around the
    environment, and computes and publishes an updated texture.

## Setting up the environment

The following environmental variables should be set up (please adjust
them for your particular configuration):

    export ASTROBEE_WS=$HOME/astrobee
    export ASTROBEE_SOURCE_PATH=$ASTROBEE_WS/src
    export ISAAC_WS=$HOME/isaac

## Robot sensors

This software makes use of three sensors that are mounted on the front
face of the robot:

   - haz cam: A low-resolution depth and intensity camera
   - nav cam: A wide field of view medium-resolution navigation camera
   - sci cam: A high-resolution narrow field of view "science" camera

The nav cam is used to determine robot's position and orientation as
it explores its environment.

Based on this, and the transform between the nav and haz cameras, the
haz cam depth readings are fused into a dense 3D model of the
environment.

The sci cam is then used to texture this model. Alternatively, the nav
cam pictures can be used for the texture as well.

An important note here is that the haz cam takes measurements about
five times per second, the nav cam perhaps twice per second, while the
science camera is triggered by an explicit command to take a picture,
unless set in continuous picture-taking mode. This aspect of the sci
cam will be elaborated later on.

## Actual vs simulated robot data

To be able to obtain high-fidelity results based on fusing the
readings of these sensors, as described earlier, a lot of careful work
needs to be done, that we discuss later in much detail. In particular,
the intrinsics of all the cameras must be calibrated accurately, the
transforms between their poses must be found, a sparse map based on
the nav cam data must be computed and registered, and various
conversions and interpolations (in time) between these sensors must be
computed.

These problems are greatly simplified with simulated data, when we
assume perfectly known cameras, including their intrinsics, how they
relate to each other, and their poses in space at any time. Hence,
working with simulated data makes it easy to test core aspects of this
sensor fusion functionality in a simplified setting.

The drawback is that having two such sources of input data complicates
the presentation of these tools. Some of them will be relevant only
for one of the two data types, and some tools that apply to both may
have different invocations for each case. The reader is advised to
keep close attention to this, and we will make it clear at every step
about which of the two paradigms one refers to.

## Sensors present in simulation

The simulator supports the ``nav_cam``, ``sci_cam``, ``haz_cam``
cameras, which are analogous to the ones on the real robot, and also
the ``heat_cam`` and ``acoustics_cam`` cameras which exist only in
simulation. All these have been tested with the geometry mapper and
streaming mapper.

The ``sci_cam`` and ``haz_cam`` cameras are not enabled by default in
the simulator. To enable them, edit the simulation configuration, in

    $ASTROBEE_SOURCE_PATH/astrobee/config/simulation/simulation.config

and set:

    haz_cam_rate = 1.0;
    sci_cam_rate = 1.0;
    sci_cam_continuous_picture_taking = true;

The later will make sure sci cam pictures are taken automatically. If
custom behavior is desired, see:

    $ISAAC_WS/src/astrobee/behaviors/inspection/readme.md

More information about the simulated nav_cam, haz_cam, and sci_cam is
at:

    $ASTROBEE_SOURCE_PATH/simulation/readme.md

The heat camera is described in:

    $ISAAC_WS/src/astrobee/simulation/isaac_gazebo/readme.md

The acoustics camera and how to enable it is documented at:

    $ISAAC_WS/src/astrobee/simulation/acoustics_cam/readme.md

## Compiling the software

It is assumed that by now the Astrobee and ISAAC software is compiled.

This module depends on two additional pieces of software, Voxblox and
CGAL.

### Compiling VoxBlox

To compile Voxblox, clone

    https://github.com/oleg-alexandrov/voxblox/

(branch master). This fork differs from the main repository at
https://github.com/ethz-asl/voxblox by the introduction of a small
tool named batch_tsdf.cc that reads the clouds to fuse and the
transforms from disk and writes the output mesh back to disk, instead
of using ROS. It also can take into account a point's reliability when
fusing the clouds, which is computed by the geometry mapper.

Compile it using the instructions at:

    https://github.com/oleg-alexandrov/voxblox/blob/master/docs/pages/Installation.rst

This should end up creating the program:

    $HOME/catkin_ws/devel/lib/voxblox_ros/batch_tsdf

### Compiling CGAL

Compile the CGAL tools following the instructions at:

    https://github.com/oleg-alexandrov/cgal_tools

This should create some programs in:

    $HOME/projects/cgal_tools
 
### CGAL license    

CGAL is released under the GPL. Care must be taken to not include it
or to link to it in any ISAAC code. Using CGAL as standalone tools
does not infringe upon GPL.

### Functionality provided by CGAL

The tools just compiled are used for smoothing meshes, filling holes,
remeshing, removing small connected components, and simplifying the
mesh. The geometry mapper can work even without these, but they
produce nicer results. The geometry mapper uses all of them except the
experimental remeshing tool.

## Data acquisition

### With real data

Acquire a bag of data on the bot. The current approach is to use a
recording profile. A step-by-step procedure is outlined below if a
recording profile has not been set up.

First give the bot the ability to acquire intensity data with the
depth camera (haz_cam). For that, connect to the MLP processor of the
bot. Edit the file:

    /opt/astrobee/config/cameras.config

Go to the section:

    picoflexx = {
      api_key = "",

and insert between the quotes the magic number which is not documented
as it is not public. If that is not done, depth point clouds will
still be produced, but without the intensity.

Every time the software on the robot is reinstalled this number get
forgotten, so it needs to be put back.

Next, open several connections to the LLP processor of astrobee as:

    ssh astrobee@llp_ip_address

In the first terminal on the bot, run:

    roslaunch astrobee astrobee.launch mlp:=mlp llp:=llp

In the second one, run:

    roslaunch /opt/astrobee/share/pico_driver/launch/pico_proxy.launch

In another terminal on LLP start the sci cam, as described in:

    $ASTROBEE_SOURCE_PATH/submodules/android/gs_examples/sci_cam_image/readme.md

If you acquire data for calibration, or even for mapping, you may want
to turn on continuous picture taking for the sci cam as described
there.

Note: It is easier to use adb control the camera, per the instructions
at the bottom of that document, than by using the guest science
manager.

In another terminal on LLP, do

    cd /data/bags
    rosbag record /hw/cam_nav /hw/depth_haz/points                   \
      /hw/depth_haz/extended/amplitude_int /hw/cam_sci/compressed

Scan a wall or a larger area with the bot facing the wall as much as
possible, and carefully scan objects jutting out of the wall. Later
one can also acquire images with various camera positions and
orientations, which help build an accurate sparse map, but those
should not be used when fusing the depth clouds or in texturing.

Copy the resulting bag off the robot.

### With simulated data

The astrobee simulator supports a handful of cameras, mentioned
earlier in the text.

#### Recording simulated data

Start the simulator, such as:

    source $ASTROBEE_WS/devel/setup.bash
    source $ISAAC_WS/devel/setup.bash
    roslaunch isaac sim.launch rviz:=true      \
      pose:="11.0 -7.0 5.0 0 0 0 1" world:=iss

In rviz turn on visualizing the desired cameras cameras from the
Panels menu, as otherwise the needed camera topics may not be
published.

Adjust the 3D view in rviz with the mouse so that the robot, which is
present in the middle of the module, can be seen.

When recording simulated data for a given camera, for example, for
``sci_cam``, so that later it can be used with the geometry and
streaming mapper, one needs to record, in addition to depth clouds and
camera images, also the camera poses and intrinsics information, which
is done as follows:

    rosbag record /hw/depth_haz/points                           \
      /hw/cam_sci/compressed /sim/sci_cam/pose /sim/sci_cam/info

It is good to check for the precise name of the camera image topic.
For haz cam the image topic will be instead:

    /hw/depth_haz/extended/amplitude_int

For nav cam, the image topic will be:

    /hw/cam_nav

and the same convention is followed for the rest of the cameras.

If desired to record data for many cameras, these topics must
be specified for each of them. For example, for ``heat_cam`` add
the lines:

    /hw/cam_heat /sim/heat_cam/pose /sim/heat_cam/info

to the ``rosbag record`` command.

The robot can be told to move around by either running a plan, or by
sending it a move command, such as:

    rosrun mobility teleop -move -pos "11.0 -5.0 5.0" -tolerance_pos 0.0001 -att "0 0 0 1"

when it will go along the module axis while not changing its orientation.

## Data pre-processing

This applies only to real data.

If the recorded data is split into many small bags, as it often happens on the
ISS, those bags should be first merged as documented in:

    $ASTROBEE_SOURCE_PATH/localization/sparse_mapping/readme.md

In order to save bandwidth, the sci cam images are published at a
reduced resolution (usually 640x480 pixels), while the full-resolution
pictures are saved to disk in a directory on HLP. That directory needs
to be fetched (see the sci cam documentation for details) and the
pictures integrated into the bag with the command:

    $ISAAC_WS/devel/lib/geometry_mapper/add_sci_cam_to_bag \
        -input_bag <input.bag> -output_bag <output.bag>    \
        -sci_cam_dir <sci_cam_dir>                         \
        -sci_cam_topic /hw/cam_sci/compressed

Any images in the input bag on the sci cam topic (presumably previews)
will be removed before the images from the sci cam directory are
appended. Only the images from the directory whose timestamp is within
the time range of what is in the bag already will be added.

EXIF metadata from the sci cam images (exposure, ISO sensitivity,
aperture, and focal length) will be saved in the bag file as well.

(Normally the sci cam data will be downloaded from the HLP of ISS
robots using some established infrastructure. Alternatively, one can
use ``adb pull``. After this tool is used, the data can be manually
deleted from HLP by first connecting to it with ``adb shell``.)

In order to run camera_calibrator, the sci cam data in the bag needs to
be decompressed, resized to 1/4 of the resolution, and made to be
grayscale. This is needed since the sci cam images are a little blurry
and, at full resolution, the corners of the calibration target are
hard to detect. The calibrator also does not handle the color format
in the bag, hence the switch to grayscale images.

However, both the geometry and the streaming mapper, as well as
camera_refiner, can handle both color and grayscale images, and both
at reduced resolution and full-resolution. These tools can adjust for
the fact that the calibration was done at reduced resolution.

To accomplish this processing, once the sci cam data is integrated
into the bag, one can do the following: 

    $ISAAC_WS/devel/lib/geometry_mapper/scale_bag --input_bag input.bag \
      --output_bag output.bag --image_type grayscale --scale 0.25

Note that the processed sci cam images will be now on topic
``/hw/cam_sci2``.

## Camera calibration

Currently the calibrator solution is not that accurate. It is suggested
to use instead camera_refiner (see further down) on a bag acquired
without a calibration target.

Camera calibration is an advanced topic. Likely your robot's cameras
have been calibrated by now, and then this step can be skipped.

This step applies only to real data and not to simulated data.

Data for calibration of the cameras should be acquired and processed
as described earlier, to reduce the image resolution and to make them
grayscale. The sci cam should be configured beforehand to take
pictures continuously.

Traditionally, the robot cameras have been calibrated using Kalibr,
using the instructions at 

    $ASTROBEE_SOURCE_PATH/scripts/calibrate/readme.md

While Kalibr returns good results for calibrating the nav cam, it is
unable to calibrate the depth portion of the haz cam, and cannot model
the fact that there may be a systematic offset between the time stamps
recorded by the various cameras (the reason for this offset is because
the published time can differ slightly than the true time at which the
image is taken).

It is recommended to use instead the calibrator provided with this
software, which can take care of those issues. The procedure to use it
is as follows.

First invoke Kalibr to save to disk the corners of the calibration
target it detects from the bag. That can be done as follows. (This
builds upon the instructions used in the doc referenced right above.)

    source $KALIBR_WS/devel/setup.bash
    rosrun kalibr kalibr_calibrate_cameras                                           \
      --topics /mgt/img_sampler/nav_cam/image_record                                 \
               /hw/depth_haz/extended/amplitude_int                                  \
               /hw/cam_sci2                                                          \
      --models pinhole-fov pinhole-radtan pinhole-radtan                             \
      --target $ASTROBEE_SOURCE_PATH/scripts/calibrate/config/granite_april_tag.yaml \
      --bag calibration.bag --dont-show-report --verbose                             \
      --target_corners_dirs calib_nav calib_haz calib_sci

Note that above we assume that the image sampler was used to collect a
subset of the nav cam images. Otherwise the nav cam topic would be
``/hw/cam_nav``.

This will create three directories with the corners extracted from the
nav, haz, and sci cameras.

Next, set up the environment:

    export ASTROBEE_RESOURCE_DIR=$ASTROBEE_SOURCE_PATH/astrobee/resources
    export ASTROBEE_CONFIG_DIR=$ASTROBEE_SOURCE_PATH/astrobee/config
    export ASTROBEE_WORLD=iss
    export ASTROBEE_ROBOT=bsharp2

Note the name of the robot, in this case it is 'bsharp2'. When the
calibrator is invoked, it will look for the configuration file named

    $ASTROBEE_SOURCE_PATH/astrobee/config/robots/bsharp2.config

and will modify it in-place. If you would like to calibrate a new
robot, first create the corresponding config file by necessarily
making a copy of an existing one.

The calibrator program is at: 

    $ISAAC_WS/devel/lib/geometry_mapper/camera_calibrator

It can be used to calibrate the intrinsics of the nav and haz camera
pair, and then of the sci and haz pair. These are referred to as
``cam1`` and ``cam2`` below. 

It is important to remember that the haz cam records both an image
intensity and a depth point cloud.

Its options are:
    
    --cam1_name: A string. The name of the first camera. Must be
        'nav_cam' or 'sci_cam'.

    --cam2_name: A string. The name of the second camera. Must be
        'haz_cam'.

    --cam1_dir: A string. The directory having the corners extracted
        from cam1 images using Kalibr.

    --cam2_dir: A string. The directory having the corners extracted
        from cam2 images using Kalibr.

    --bag: A string. The ros bag having the calibration data.

    --haz_cam_points_topic. A string. The depth point cloud topic in
        the bag file. The default is /hw/depth_haz/points.

    --start: A double. The starting time in seconds from which to
        process the bag. The default is 0.0 (beginning of the
        bag).

    --duration: A double. Specify how many seconds from the bag to
        process. The default is -1 (process the whole bag).

    --update_cam1: A boolean. Update the intrinsics of cam1. The
        default is 'false'.

    --update_cam2: A boolean. Update the intrinsics of cam2. The
        default is 'false'.

    --update_depth_to_image_transform: A boolean. Update the transform
        to apply to scale correctly the depth camera clouds. The
        default is 'false'.

    --update_extrinsics: A boolean. Update the transform from cam2 to
        cam1. The default is 'false'.

    --timestamp_offset_sampling: A string. If specified, as 'beg end
        num', create num samples between beg and end, and determine
        the timestamp offset between these cameras as the sample
        minimizing the extrinsics error among cam1 and cam2.

    --cam1_intrinsics_to_float: A string. Refine 0 or more of the
        following intrinsics for cam1: focal_length, optical_center,
        distortion. Specify as a quoted list. For example: 
        'focal_length optical_center'.
  
    --cam2_intrinsics_to_float: A string. Refine 0 or more of the
        following intrinsics for cam2: focal_length, optical_center,
        distortion. Specify as a quoted list. For example: 
        'focal_length optical_center'.

    --num_cam1_focal_lengths: An integer. If set to 2, use separate
        focal lengths along image rows and columns for cam1. Else use
        the same one. The default is 1.

    --num_cam2_focal_lengths: An integer. If set to 2, use separate
        focal lengths along image rows and columns for cam2. Else use
        the same one. The default is 1.

    --num_cam1_iterations: An integer. How many solver iterations to
        perform to solve for cam1 intrinsics. The default is 10000.

    --num_cam2_iterations: An integer. How many solver iterations to
        perform to solve for cam2 intrinsics. The default is 10000.

    --num_extrinsics_iterations: An integer. How many solver
        iterations to perform to solve for extrinsics. The default is
        10000.

    --robust_threshold: A double. Pixel errors much larger than this
        will be exponentially attenuated to affect less the cost
        function. This should not be too low, as if the initial guess
        is off, initial pixel errors can be rather large. The default
        is 4.0.

    --parameter_tolerance: A double. Stop when the optimization
        variables change by less than this. The default is 1e-12.

    --calib_num_ransac_iterations: An integer. Use in this many RANSAC
        iterations to find camera poses based on calibration target
        measurements. The default is 1000.

    --calib_ransac_inlier_tolerance: An integer. Use this inlier
        tolerance (in pixels) to find camera poses. The default is 10.

    --max_interp_dist: A double. If two consecutive camera poses have
        timestamps that differ more than this, do not interpolate
        between them. The default is 4.0.

    --max_haz_cam_image_to_depth_timestamp_diff: A double. Use depth
        haz cam clouds that are within this distance in time from the
        nearest haz cam intensity image. The default is 0.2.

    --output_dir: A string. If provided, write here the residuals
        (differences between pixel target corners and projection into
        cameras of measured target points) before and after
        optimization.

An example of calibrating the nav and haz cam pairs. (Ensure that the
robot name was set as described earlier, and keep in mind that this
tool will modify robot's config file in place.)

    $ISAAC_WS/devel/lib/geometry_mapper/camera_calibrator                   \
        --start 0 --duration 300 --cam1_name nav_cam --cam1_dir calib_nav   \
        --cam2_name haz_cam --cam2_dir calib_haz                            \
        --haz_cam_points_topic /hw/depth_haz/points --bag mybag.bag         \
        --update_cam2                                                       \
        --cam2_intrinsics_to_float 'focal_length optical_center'            \
        --num_cam2_iterations 10000 --num_cam2_focal_lengths 1              \
        --update_extrinsics                                                 \
        --update_depth_to_image_transform                                   \
        --timestamp_offset_sampling '-0.03 -0.01 5'

The last two options better not be used at all, or with a lot of
care. 

This tool can return an incorrect solution for distortion unless a lot
of care is taken to have many target points at the periphery of the
sensor. If in doubt, that parameter better be kept fixed. The camera
refiner can refine the sci cam intrinsics and will likely do a better
job.

It was found experimentally that the depth to image transform
which is updated by ``--update_depth_to_image_transform``
depends very much on how far the calibration target is from the
camera. The value of ``hazcam_depth_to_image_transform`` already in the
robot config file, which shows roughly a scale transform with a factor
of 0.95 is good enough.

One has to be also be careful with the option
``--timestamp_offset_sampling``, and even avoid using it in a first
pass.
  
Notice that here we chose to not update the intrinsics of cam1 (nav_cam).
That is because this camera was calibrated with Kalibr a while ago and it
is known to be accurate. If desired to calibrate it, one can add the option
`--update_cam1``.

Only if after using the geometry mapper one notices visible
registration errors (which manifest themselves as artifacts in the
texture output by this tool), should one consider tweaking the
parameters controlled by these options.

The next step is to calibrate the sci cam and haz cam pairs, which 
can go as follows:

    $ISAAC_WS/devel/lib/geometry_mapper/camera_calibrator                   \
        --start 0 --duration 300 --cam1_name sci_cam --cam1_dir calib_sci   \
        --cam2_name haz_cam --cam2_dir calib_haz                            \
        --haz_cam_points_topic /hw/depth_haz/points --bag mybag.bag         \
        --update_cam1                                                       \
        --cam1_intrinsics_to_float 'focal_length optical_center'            \
        --num_cam1_iterations 10000 --num_cam1_focal_lengths 1              \
        --update_extrinsics                                                 \
        --timestamp_offset_sampling '-0.35 -0.15 11'

As before, one better not use the option ``--timestamp_offset_sampling``
unless one is sure it is necessary.

Note that this time we optimize the intrinsics of cam1 (sci_cam)
and we do not use ``--update_depth_to_image_transform`` or optimize
the intrinsics of cam2 (haz_cam) as this was already done earlier.
We do not optimize the distortion of cam1 as that can result in
incorrect values if there are not enough measurements at image periphery.
The distortion is best optimized with the camera refiner (see below).

Optionally, one can try to further refine the calibration if not good
enough when used with data collected while the bot is flying around
doing inspection. This is an advanced topic which is handled further
down this document, in the section on camera refinement.

## Data extraction

Nav cam images can be extracted from a bag as follows:

    $ASTROBEE_WS/devel/lib/localization_node/extract_image_bag       \
      mydata.bag -image_topic /mgt/img_sampler/nav_cam/image_record  \
      -output_directory nav_data -use_timestamp_as_image_name

The last option, ``-use_timestamp_as_image_name``, must not be missed.
It makes it easy to look up the image acquisition based on image name,
and this is used by the geometry mapper.

One should check beforehand if the nav cam topic is correct. If the image
sampler was not used, the nav cam topic would be /hw/cam_nav.

To extract the sci cam data, if necessary, do:

    $ASTROBEE_WS/devel/lib/localization_node/extract_image_bag \
     mydata.bag -image_topic /hw/cam_sci/compressed            \
     -output_directory sci_data -use_timestamp_as_image_name

To extract the depth clouds, which may be useful for debugging purposes,
do:

    $ISAAC_WS/devel/lib/geometry_mapper/extract_pc_from_bag mydata.bag \
       -topic /hw/depth_haz/points -output_directory pc_data           \
       -use_timestamp_as_image_name

## Map building and registration

Build and register a SURF sparse map with the nav cam images. (This is
needed only with real data.) See the 
[sparse mapping](https://nasa.github.io/astrobee/html/sparsemapping.html) 
documentation in the Astrobee repository, with more details given in
the [map building](https://nasa.github.io/astrobee/html/map_building.html)
page.  

If the map to be built is large, consider using the Theia SfM
software. See the [Theia documentation](https://nasa.github.io/astrobee/html/theia_map.html)
for how to use this package to create Astrobee sparse maps.

An example for how to build a map in the context of calibration is
also given further down this document.

This SURF map will be used with the geometry mapper. Rebuild it with
BRISK features, to be used with the streaming mapper. Examine the
BRISK obtained map. If it does not have enough features, rebuild it
with a lower value of ``--default_brisk_threshold`` and
``--max_brisk_threshold`` (For example, use 70 instead of the default of
90. This may make the sparse map bigger.)

It is suggested to not use the ``--histogram_equalization`` flag for the
SURF map, but to use it with the BRISK map.

Don't forget to set:

    export ASTROBEE_ROBOT=<robot name>

and the other environmental variables from that document before running
map-building.

It is very important to keep in the map at least one nav cam image
whose timestamp is a couple of seconds before the sci cam image that
we would like to later overlay on top of the mesh created with with
the help of this map, and one nav cam image a couple of seconds after
the last sci cam image we would like to keep, to ensure we process all
sci cam images with the geometry mapper.

# Running the geometry mapper

## When using real data

The geometry mapper fuses the depth cloud data and creates textures
from the image cameras.

Any image camera is supported, as long as present in the robot
configuration file and a topic for it is in the bag file (see more
details further down). The geometry mapper can handle both color and
grayscale images, and, for sci cam, both full and reduced resolution.

Ensure that the bot name is correct below. Set ``ASTROBEE_SOURCE_PATH``,
`ASTROBEE_WS``, and ``ISAAC_WS`` as earlier. Run:

    export ASTROBEE_RESOURCE_DIR=$ASTROBEE_SOURCE_PATH/astrobee/resources
    export ASTROBEE_CONFIG_DIR=$ASTROBEE_SOURCE_PATH/astrobee/config
    export ASTROBEE_WORLD=iss
    export ASTROBEE_ROBOT=bsharp2
    source $ASTROBEE_WS/devel/setup.bash
    source $ISAAC_WS/devel/setup.bash
    python $ISAAC_WS/src/dense_map/geometry_mapper/tools/geometry_mapper.py \
      --ros_bag data.bag                                                    \
      --sparse_map nav_data.map                                             \
      --camera_types "sci_cam nav_cam haz_cam"                              \
      --camera_topics "/hw/cam_sci/compressed /mgt/img_sampler/nav_cam/image_record /hw/depth_haz/extended/amplitude_int"\
      --undistorted_crop_wins "sci_cam,1250,1000 nav_cam,1100,776 haz_cam,250,200" \
      --haz_cam_points_topic /hw/depth_haz/points                           \
      --start 0                                                             \
      --duration 1e+10                                                      \
      --sampling_spacing_seconds 5                                          \
      --dist_between_processed_cams 0.1                                     \
      --angle_between_processed_cams 5.0                                    \
      --depth_exclude_columns 0                                             \
      --depth_exclude_rows 0                                                \
      --foreshortening_delta 5.0                                            \
      --median_filters "7 0.1 25 0.1"                                       \
      --reliability_weight_exponent 2                                       \
      --voxblox_integrator merged                                           \
      --depth_hole_fill_diameter 30.0                                       \
      --max_ray_length 2.5                                                  \
      --voxel_size 0.01                                                     \
      --max_iso_times_exposure 5.1                                          \
      --smoothing_time 5e-6                                                 \
      --max_num_hole_edges 8000                                             \
      --max_hole_diameter 0.3                                               \
      --num_min_faces_in_component 100                                      \
      --num_components_to_keep 100                                          \
      --edge_keep_ratio 0.2                                                 \
      --output_dir data_dir                                                 \
      --verbose

Parameters:

    --ros_bag: A ROS bag with recorded image and point cloud data.
    --sparse_map: A registered SURF sparse map of a subset of the nav_cam data.
    --camera_types: Specify the cameras to use for the textures, as a list
      in quotes. Default: "sci_cam nav_cam haz_cam". With simulated
      data only ``sci_cam`` is supported.
    --camera_topics: Specify the bag topics for the cameras to texture
      (in the same order as in ``--camera_types``). Use a list in quotes.
      The default is in the usage above.
    --undistorted_crop_wins: The central region to keep after
      undistorting an image and before texturing. For sci_cam the
      numbers are at 1/4th of the full resolution (resolution of
      calibration) and will be adjusted for the actual input image
      dimensions. Use a list in quotes. The default is
      "sci_cam,1250,1000 nav_cam,1100,776 haz_cam,250,200".
    --haz_cam_points_topic: The depth point cloud topic in the bag file.
    --start: How many seconds into the bag to start processing the data.
    --duration: For how many seconds to do the processing.
    --sampling_spacing_seconds: How frequently to sample the sci and haz 
      cameras, in seconds. The default is 2.
    --dist_between_processed_cams: Once an image or depth cloud is processed, 
      process a new one whenever either the camera moves by more than this 
      distance, in meters, or the angle changes by more than 
      --angle_between_processed_cams, in degrees. The default is 0.1.
    --angle_between_processed_cams: See --dist_between_processed_cams. The
      default is 5.0. 
    --sci_cam_timestamps: Process only these sci cam timestamps (rather than 
      any in the bag using --dist_between_processed_cams, etc.). Must be 
      a file with one timestamp per line. 
    --depth_exclude_columns: Remove this many columns of data from the 
      rectangular depth image sensor at margins to avoid distortion.
      The default is 0.
    --depth_exclude_rows: Remove this many columns of data from the 
      rectangular depth image sensor at margins to avoid distortion.
      The default is 0.
    --foreshortening_delta: A smaller value here will result in holes
      in depth images being filled more aggressively but potentially
      with more artifacts in foreshortened regions.
    --median_filters: Given a list "w1 d1 w2 d2 ... ", remove a depth
      image point if it differs, in the Manhattan norm, from the median
      of cloud points in the pixel window of size wi centered at it by
      more than di. This removes points sticking out for each such i. The
      default is "7 0.1 25 0.1".
    --depth_hole_fill_diameter: Fill holes in the depth point clouds
      with this diameter, in pixels. This happens before the clouds
      are fused. It is suggested to not make this too big, as more
      hole-filling happens on the fused mesh later (see
      --max_hole_diameter). The default is 30.
    --reliability_weight_exponent: A larger value will give more
      weight to depth points corresponding to pixels closer to depth
      image center, which are considered more reliable. The default is
      2.0.
    --max_ray_length_m: Dictates at what distance from the depth camera
      to truncate the rays (in meters). This can be small if the images
      are acquired close to walls and facing them.
    --voxblox_integrator: When fusing the depth point clouds use
      this VoxBlox method. Options are: "merged", "simple", and
      "fast". The default is "merged".
    --voxel_size: The grid size used for binning depth cloud points and 
      creating the mesh. Measured in meters.
    --max_iso_times_exposure: Apply the inverse gamma transform to
      images, multiply them by max_iso_times_exposure/ISO/exposure_time
      to adjust for lightning differences, then apply the gamma
      transform back. This value should be set to the maximum observed
      ISO * exposure_time. The default is 5.1. Not used with simulated data.
    --smoothing_time: A larger value will result in a smoother mesh. The default 
      is 0.00005.
    --max_num_hole_edges: Close holes in the mesh which have no more than this 
      many edges. The default is 1000.
    --max_hole_diameter: The diameter (in meters) of the largest hole in the
      mesh to fill. The default is 0.3.
    --num_min_faces_in_component: Keep only connected mesh components with 
      at least this many faces.
    --num_components_to_keep: How many of the largest connected components 
      of the mesh to keep. Being too aggressive here can result in a mesh 
      with missing parts. The default is 10.
    --edge_keep_ratio: Simply the mesh keeping only this fraction of
      the original edges. The default is 0.2.
    --output_dir: The directory where to write the processed data.
    --simulated_data: If specified, use data recorded in simulation. 
      Then haz and sci camera poses and intrinsics should be recorded in the 
      bag file (see earlier about recording simulated data).
    --localization_options: Options to to use to localize the nav cam images.
      These mostly relate to the SURF (or BRISK parameters). Default: 
      '--min_surf_features 400 --max_surf_features 600 --min_surf_threshold 5 -default_surf_threshold 10 --max_surf_threshold 1000 --early_break_landmarks 400 -verbose_localization'. 
    --use_brisk_map: If this flag is specified, instead of a SURF sparse map 
      made from the same bag that needs texturing, use a pre-existing and 
      unrelated BRISK map. This map may be more convenient but less
      reliable. In this case one must specify carefully the range of
      times in the bag to use as it will no longer be constrained by
      the timestamps in the map.
    --start_step: Start processing at this step. Useful for resuming
      work. Values: 0 (determine poses), 1 (fuse meshes), 2 (smoothe
      mesh), 3 (fill holes), 4 (clean mesh and rm small connected
      components), 5 (smoothe again), 6 (fill holes again), 7 (smoothe again),
      8 (simplify the mesh), 9 (texture mesh).
    --merge_maps: Given several output geometry mapper directories, specified 
      as a list in quotes, create a merged textured mesh. The input
      bag and sparse map will not be used. Each input geometry mapper run
      can have its own bag and sparse map. The sparse maps must be
      registered to a global coordinate system and co-registered to
      each other, such as when extracted from a larger merged and
      registered map.
    --external_mesh: Use this mesh to texture the images, rather than
      creating one from depth data in the current bag.
    --nav_cam_to_sci_cam_offset_override_value: Override the
      value of nav_cam_to_sci_cam_timestamp_offset from the robot config
      file with this value.
    --verbose: If specified, echo all output in the terminal.
    --texture_individual_images: If specified, in addition to a joint texture 
      of all images create individual textures for each image and camera. Does 
      not work with simulated cameras. For debugging.
    --save_debug_data: If specified, save many intermediate datasets
      for debugging.

The outputs produced by this tool are four meshes that can be visualized
with Meshlab:

  - data_dir/fused_mesh.ply: A mesh obtained by fusing the haz cam depth point clouds.
  - data_dir/smooth_mesh.ply: A smoothed version of the mesh.
  - data_dir/hole_filled_mesh.ply: A mesh obtained by filling holes in the fused mesh.
  - data_dir/clean_mesh.ply: The mesh with small connected components removed.
  - data_dir/smooth_mesh2.ply: A further smoothed version of the mesh.
  - data_dir/hole_filled_mesh2.ply: A further hole-filled mesh.
  - data_dir/simplified_mesh.ply: The simplified mesh.
  - data_dir/smooth_mesh3.ply: A further smoothed version of the mesh.
  - data_dir/sci_cam_texture/run.obj: The mesh overlayed with the sci cam texture.
  - data_dir/nav_cam_texture/run.obj: The mesh overlayed with the nav cam texture.
  - data_dir/haz_cam_texture/run.obj: The mesh overlayed with the haz cam texture.

(Several passes of smoothing and hole-filling, as done above, appear
necessary from experiments.)

Intermediate products are the undistorted nav cam and sci cam images.
It is suggested to review those in an image viewer, such as 'eog' and
delete some of them if they cause artifacts. After that, the last part
of the pipeline (invocation of the texrecon tool) can be redone.

To run this tool it is suggested to pick a portion of the bag where
the images face the wall as much as possible, so one may need to
change the ``--start`` and ``--duration`` values.

Unless the flag ``--use_brisk_map`` is set, the data processing will be
restricted to the range of timestamps contained within the sparse map
(this is another restriction, in addition to ``--start`` and
`--duration``).

If this tool is too slow, or if localization fails, consider adjusting
the ``--localization_options`` above. For example, to make localization
work better (which will make the program slower) decrease the value of
``--default_surf_threshold``, and increase ``--early_break_landmarks``,
``--min_surf_features``, and ``--max_surf_features``. To make it faster,
do the opposite. 

The values of ``--depth_exclude_columns`` and ``--depth_exclude_rows``
can be adjusted to remove rows and columns at the margins
which may result in a nicer mesh. If, for example, the bot
moves upwards or downwards, there is little loss in removing
some rows at the margins as they will be filled in from
other depth readings. Then, if there are several up-down scans
with good lateral overlap, removing some columns won't result
in much loss but may remove some noise. 

If it is desired to use only a precise subset of the sci cam images,
specify those with the option ``--sci_cam_timestamps``.

If several acquisitions were performed, and the geometry mapper was
run with each of them, those can be merged by invoking the
geometry mapper with the option ``--merge_maps``.

The geometry mapper can run with a previously created mesh if invoked
with the option ``--external_mesh``. 

The most time-consuming part of the geometry mapper is computing the
initial poses, which is the earliest step, or step 0. To resume the
geometry mapper at any step, use the option ``--start_step num``. For
example, one may want to apply further smoothing to the mesh or more
hole-filling, before resuming with the next steps.

For a given camera type to be textured it must have entries in
``cameras.config`` and the robot config file (such as
``bumble.config``), which are analogous to existing
``nav_cam_to_sci_cam_timestamp_offset``,
``nav_cam_to_sci_cam_transform``, and ``sci_cam`` intrinsics, with
"sci" replaced by your camera name. The geometry mapper arguments
``--camera_types``, ``--camera_topics``, and
``--undistorted_crop_wins`` must be populated accordingly, with some
careful choice to be made for the last one. Images for the desired
camera must be present in the bag file at the the specified topic.

## With simulated data

The geometry mapper works with any simulated cameras not having
distortion. It was tested to work with simulated images for
``sci_cam``, ``haz_cam``, ``heat_cam``, and ``acoustics_cam``. It does
not work with ``nav_cam``, which has distortion.

The flag:

    --simulated_data

should be passed to the geometry mapper. The sparse map is not
necessary, no localization will take place, and intrinsics
information, camera transform, and timestamp offset will not be read
from the robot configuration file. Instead, it is expected that each
simulated camera, for example ``sci_cam``, will provide, in addition to an
image topic, the topics

    /sim/sci_cam/pose
    /sim/sci_cam/info

having the pose and intrinsics of each camera image. These should be
recorded in the bag (see more on recording earlier in the document).

It is assumed that the simulated images are not distorted. In particular,
``nav_cam``, which has fisheye lens distortion, is not supported. 

The simulated haz_cam is required to be among the topics being recorded
and read in, because its pose is needed to process the depth clouds.

Example of running the geometry mapper with simulated data:

    sci_topic=/hw/cam_sci/compressed 
    haz_topic=/hw/depth_haz/extended/amplitude_int
    python $ISAAC_WS/src/dense_map/geometry_mapper/tools/geometry_mapper.py \
      --simulated_data                                                      \
      --ros_bag data.bag                                                    \
      --camera_types "sci_cam haz_cam"                                      \
      --camera_topics "$sci_topic $haz_topic"                               \
      --haz_cam_points_topic /hw/depth_haz/points                           \
      --output_dir data_dir                                                 \
      --sampling_spacing_seconds 2                                          \
      --dist_between_processed_cams 0.1                                     \
      --angle_between_processed_cams 5.0                                    \
      --verbose

It is important to check for the correct names for the camera image
topics are passed to ``--camera_topics``.

## Running the streaming mapper

Here we assume that the geometry mapper ran and created a dense 3D
model of the region of interest. Then, the robot is run, whether in
simulation or in the real world. It records images with a camera,
which can be sci cam, nav cam, etc. (see the complete list below),
that we call the "texture images".

The streaming mapper ROS node will then overlay each texture image
received with this camera on top of the 3D model, and publish the
obtained textured model to be visualized. 

### Running with real data

#### When the robot (or nav cam) poses are known

To run the streaming mapper with real data for the given bot, do:

    source $ASTROBEE_WS/devel/setup.bash
    source $ISAAC_WS/devel/setup.bash
    export ASTROBEE_RESOURCE_DIR=$ASTROBEE_SOURCE_PATH/astrobee/resources
    export ASTROBEE_CONFIG_DIR=$ASTROBEE_SOURCE_PATH/astrobee/config
    export ASTROBEE_WORLD=iss
    export ASTROBEE_ROBOT=bsharp2
    roslaunch isaac isaac_astrobee.launch llp:=disabled nodes:=framestore \
      streaming_mapper:=true output:=screen     

Wait until it finishes forming the texture model, which may take 
30 seconds on more.

Ensure that the ``ASTROBEE_ROBOT`` name is correct above.

This node will load the mesh produced by the geometry mapper from 

    $ISAAC_WS/src/isaac/resources/geometry/${ASTROBEE_WORLD}.ply

If an updated geometry mapper mesh exists, it should be copied to that
location first.

The tool will read the configuration options from:

    $ISAAC_WS/src/isaac/config/dense_map/streaming_mapper.config

By default, as specified in that file, it will listen to the robot
body pose being published at:

    /gnc/ekf

That config file specifies the the camera image to texture, which, by
default, is ``sci_cam``. Its image topic must be set, which, for
``sci_cam`` is normally

    /hw/cam_sci/compressed

while for ``haz_cam`` is:

    /hw/depth_haz/extended/amplitude_int

For ``nav_cam`` the image topic is either

    /mgt/img_sampler/nav_cam/image_record

if the images are played from a bag which recorded the images produced
with the image sampler, or

    /hw/cam_nav

if no image sampler was used. The rest of the cameras usually follow
the convention on the line above.

For the sci cam, the streaming mapper also expects image exposure data
on the topic:

    /hw/sci_cam_exif

to do exposure correction, unless in simulation mode or
``sci_cam_exposure_correction`` is set to false. This is not needed
for other cameras.

In order for the streaming mapper to produce texture it should receive
input information with its needed topics (robot or nav_cam pose and
the image to texture), either from a bag or from the robot in real
time. A data bag can be played with the usual command:

    rosbag play data.bag

See the note further down if it is desired to rename the topics
on which some bag data is published.

The streaming mapper will publish several topics having texture
information, which for ``sci_cam`` are:

    /ism/sci_cam/img
    /ism/sci_cam/obj
    /ism/sci_cam/mtl

and an analogous convention is followed for other cameras.

The header.stamp value for each published message will be the same as
the header.stamp for the corresponding input camera image.

The data produced by the streaming mapper can be recorded (for
sci_cam, and analogously for other cameras) with:

    rosbag record /ism/sci_cam/img /ism/sci_cam/obj /ism/sci_cam/mtl \
       -b 10000
    
The recording should start before the input bag is played. The ``-b``
option tells ROS to increase its recording buffer size, as sometimes
the streaming mapper can publish giant meshes.

The robot pose that the streaming mapper needs assumes a very accurate
calibration of the IMU sensor in addition to the nav, haz, and sci cam
sensors, and very accurate knowledge of the pose of these sensors on
the robot body. If that is not the case, it is suggested to use the
nav cam pose via the ``nav_cam_pose_topic`` field in
streaming_mapper.config (set it to ``/loc/ml/features``), for which
only accurate calibration of the nav, sci, and haz cameras among each
other is assumed, while the ``ekf_pose_topic`` must be set to an empty
string.

The input texture can be in color or grayscale, at full or reduced
resolution, and compressed or not. 

#### Running with no robot or nav cam pose information

If no robot body or nav cam pose information is present, for example,
if the EKF or localization node was not running when the image data was
acquired, or this data was not recorded or was not reliable, the
localization node can be started together with the streaming mapper,
and this node will provide updated pose information.

Edit ``streaming_mapper.config`` and set ``nav_cam_pose_topic`` to
``/loc/ml/features`` and let ``ekf_state_topic`` be empty.

The localization node will make use of a registered sparse
map with BRISK features, histogram equalization, and a vocabulary
database to find the nav cam poses. The command for building such a
BRISK map from a registered SURF map is:

    cp surf_map.map brisk_map.map 
    $ASTROBEE_WS/devel/lib/sparse_mapping/build_map                 \
      --output_map brisk_map.map --rebuild --histogram_equalization \
      --vocab_db

See:

     $ASTROBEE_SOURCE_PATH/localization/sparse_mapping/readme.md

for more information.

If running on a local machine, after the map is rebuilt it should be
copied to:

    $ASTROBEE_RESOURCE_DIR/maps/${ASTROBEE_WORLD}.map

(The ``maps`` directory needs to be created if missing.)

Also see a note earlier in the text for how to reduce the BRISK
thresholds if the map has too few features. For more details on what
the localization node expects, see build_map.md, in the section about
running this node on a local machine.

Ensure the same environment as before is present, including the robot name,
and run:

    roslaunch isaac isaac_astrobee.launch mlp:=local llp:=disabled \
      nodes:=framestore,localization_node,localization_manager     \
      streaming_mapper:=true output:=screen     

Wait until the textured model is created, which can take a minute.

Then, in a different terminal, play the bag. The localization node will
expect the nav cam images to be published on topic ``/hw/cam_nav``. If
it is on a different topic, such as
``/mgt/img_sampler/nav_cam/image_record``, it needs to be redirected
to this one when playing the bag, such as:

    rosbag play --clock data.bag                         \
      /mgt/img_sampler/nav_cam/image_record:=/hw/cam_nav \
      /loc/ml/features:=/tmp/features /gnc/ekf:=/tmp/ekf

Above the /loc/ml/features and /gnc/ekf topics which may exist in the
bag are redirected to temporary topics, since the currently started
localization node will create new camera pose information.

The ``--clock`` option should not be missed.

Then enable the localization node by running in a separate
terminal:

    rosservice call /loc/ml/enable true

#### The streaming mapper configuration file

The ``streaming_mapper.config`` file has following fields:

  - mesh_file: Override the location of the mesh described earlier.
  - ekf_state_topic: The topic to listen to for robot pose information.
    It can be set to /gnc/ekf, whose type is ff_msgs::EkfState.
  - ekf_pose_topic: An alternative topic for the robot pose. It can
    be set to /loc/pose, of type geometry_msgs::PoseStamped.
  - nav_cam_pose_topic: The preferred topic for the pose. This
    is the pose of the nav cam, not of the robot, and can be set to 
    /loc/ml/features. It is published by the localization node.
  - texture_cam_type: The camera that creates the texture images
    (can be nav_cam, sci_cam, haz_cam, and in simulation also
    heat_cam and acoustics_cam). The default is sci_cam. This
    field affects the name of the topics on which the streaming
    mapper publishes its output.
  - texture_cam_topic: The topic having the images (texture) to
    overlay. The default value is /hw/cam_sci/compressed and see
    note in the text for other cameras. 
  - dist_between_processed_cams: Once an image is textured and 
    published, process a new one whenever either the camera moves by
    more than this distance, in meters, or the angle changes by more
    than angle_between_processed_cams, in degrees. The default is
    0.1.
  - angle_between_processed_cams: See: dist_between_processed_cams. 
    The default is 5.0. 
  - max_iso_times_exposure: Apply the inverse gamma transform to
    images, multiply them by max_iso_times_exposure/ISO/exposure_time
    to adjust for lightning differences, then apply the gamma
    transform back. This value should be set to the maximum observed
    ISO * exposure_time. The default is 5.1. Not used with simulated 
    data, when sci_cam_exposure_correction if false, or other cameras
    than sci_cam.
  - sci_cam_exposure_correction: If set to 'true', correct the
    exposure of sci_cam images. Read exposures from /hw/sci_cam_exif.
  - use_single_texture: If set to 'true', use a single texture
    buffer. Sample the images by picking points on each triangle face
    with spacing pixel_size. This can take a couple of minutes to form
    the necessary structures to be able to stream the texture.
  - pixel_size: The pixel size to use with use_single_texture.
    The default is 0.001 meters.
  - num_threads: Number of threads to use. The default is 48.
  - save_to_disk: If set to 'true', save to disk an .obj file for each
    topic published, to be debugged in Meshlab. These will be saved in
    ~/.ros. The default is 'false'.

## Running with simulated data

For simulated data the usage is somewhat different. First,
simulation.config needs to be edited as described earlier in the
document to turn on the simulated sci cam, haz cam, or other desired
camera.

When working with ISS data, more specifically the JPM module, do:

    export ASTROBEE_WORLD=iss
    /bin/cp -fv $ISAAC_WS/src/isaac/resources/geometry/iss_sim.ply \
      $ISAAC_WS/src/isaac/resources/geometry/${ASTROBEE_WORLD}.ply

to use the simulated mesh for this module.

Edit ``streaming_mapper.config`` and set:

    use_single_texture = false;

Otherwise, the exiting simulated mesh has so many triangles that it
will overwhelm the size of the single buffer which is meant to fit all
textures.

To launch the streaming mapper, do:

    source $ASTROBEE_WS/devel/setup.bash
    source $ISAAC_WS/devel/setup.bash
    export ASTROBEE_RESOURCE_DIR=$ASTROBEE_SOURCE_PATH/astrobee/resources
    export ASTROBEE_CONFIG_DIR=$ASTROBEE_SOURCE_PATH/astrobee/config
    export ASTROBEE_WORLD=iss
    roslaunch isaac sim.launch world:=iss rviz:=true        \
      streaming_mapper:=true pose:="11.0 -7.0 5.0 0 0 0 1"  \
      output:=screen 

It is suggested to read the documentation above for working with the
simulator in the context of the geometry mapper for more details, such
as how to ensure the desired camera images are seen in rviz.

Then, the robot can be moved in order to create images to texture as:

    rosrun mobility teleop -move -pos "11.0 -5.0 5.0" -tolerance_pos 0.0001 \
      -att "0 0 0 1"

With simulated data the pose and intrinsics for each camera are
received directly from the simulator, on topics that, for example, for
sci_cam, are:

    /sim/sci_cam/pose
    /sim/sci_cam/info

Hence, the parameters ``ekf_state_topic``, ``ekf_pose_topic``, and
``nav_cam_pose_topic`` are ignored.

The streaming mapper will publish its results on topics mentioned
earlier in the text.

Note that value of ``ASTROBEE_ROBOT`` is not needed in this case. Any
user-set value will be overwritten with the robot name ``sim``.

## Camera refinement

The calibration done with the calibration target can still leave some
residual registration error between the cameras, which manifests
itself as discrepancies in the geometry mapper and streaming mapper
products, and between the nav cam and sci cam textures.

Once a dataset of the robot flying around and performing inspections
is acquired, so in realistic conditions, rather than with a
calibration target, it can be used to further refine the camera
calibration file, including the intrinsics and extrinsics.

The calibration step above can be avoided altogether, and this robot's
desired transforms to be refined can be initialized with values from a
different robot or with the placeholder values already present in a
given robot's config file.

To avoid issues with accuracy of the timestamps of the images, we
assume that the robot is paused, or otherwise moves very slowly,
during each sci cam shot. Then, to refine the camera calibration, the
following approach should be taken.

### Image selection

Select a set of nav cam images shortly before and after each sci cam
image using the image_picker tool:

    export ASTROBEE_RESOURCE_DIR=$ASTROBEE_SOURCE_PATH/astrobee/resources
    export ASTROBEE_CONFIG_DIR=$ASTROBEE_SOURCE_PATH/astrobee/config
    export ASTROBEE_WORLD=iss
    export ASTROBEE_ROBOT=bsharp2
    source $ASTROBEE_WS/devel/setup.bash
    source $ISAAC_WS/devel/setup.bash
    $ISAAC_WS/devel/lib/geometry_mapper/image_picker                 \
      --ros_bag mybag.bag                                            \
      --nav_cam_topic /mgt/img_sampler/nav_cam/image_record          \
      --sci_cam_topic /hw/cam_sci/compressed                         \
      --haz_cam_intensity_topic /hw/depth_haz/extended/amplitude_int \
      --bracket_len 0.6 --output_nav_cam_dir nav_images

Setting up the correct robot name for ASTROBEE_ROBOT is very important.

The --bracket_len option brackets sci cam images by nav cam images so
the length of time between these two nav cam images is at most this
value. These nav cam images are saved. Later in camera_refiner sci cam
and haz cam images are picked in each such a nav cam interval. (A
camera's time is first adjusted for the timestamp offset between the
cameras.)

It is important to note that the bracket length can affect the accuracy
of calibration later, and hence it should be rather tight. Yet a tight
bracket does not allow for wiggle room if later it is desired to tweak
a little the timestamp offsets while still staying within the bounds,
and it may prevent bracketing all the sci cam images and enough haz
cam images.

The option --nav_cam_to_sci_cam_offset_override_value can be
used if the given bag is suspected to have a different value of this
offset. Such an option the option can be passed also to the camera
refiner below and to the geometry mapper.

Examine these images. Since they are used for bracketing, there will
be pairs of very similar images. Yet, it is good that there should be
no excessive overlap otherwise, so the images in the first pair better
have about 4/5 overlap with images from other pairs.

If necessary, add more intermediate images by re-running this tool
with:

    --max_time_between_images <val>

It is good to not allow too many images or excessive overlap, but, if
removing excessive images, ensure that each sci cam image is still
bracketed as originally by two nav cam images close in time, and keep
the images in pairs of very similar ones. That is good even if there's
no sci cam images between some pairs, as it is likely haz cam images
are found later in that bracket.

One could start by running this tool with a smaller value of the
--max_time_between_images option, wiping many redundant images while
ensuring there is good overlap among them and keeping pairs of similar
images, then running this tool one more time with a very large value
of this option to ensure the bracketing images for each sci cam image
are added back.

### Map building and registration

Build a sparse map with these images. Use the same environment as
above:

    dir=nav_images
    images=$(ls $dir/*jpg)
    surf_map=${dir}_surf.map
    $ASTROBEE_WS/devel/lib/sparse_mapping/build_map                 \
      --output_map $surf_map --feature_detection --feature_matching \
      --track_building --incremental_ba  --bundle_adjustment        \
      --min_valid_angle 1.0 --num_subsequent_images 20 $images 

Notice how we used "--min_valid_angle 1.0". The two images used for
each sci cam and haz cam bracketing will be very similar, and this
will prevent having too many features which result in small
convergence angles between the rays, which may make map-building less
stable.

If the map makes a closed loop, and, for example, image 80 becomes
similar to image 0, one should increase --num_subsequent_images to
perhaps 90. This would result in increased runtime but a better map.

Register the map. That can be done, for example, by merging this map
with a registered map, bundle-adjusting the obtained map,
re-registering it, then extracting the submap corresponding to the
original images without re-bundle-adjusting the extracted submap to
maintain the same coordinate system as after registration.

This process can be time-consuming. A trick which can speed things up
is to extract a smaller but representative submap from the registered
map, say showing all the walls as our map, and merge and register our
map to this registered submap. That goes as follows.

Examine the existing registered map in nvm_visualize and record in a
list named ``list.txt`` the images which are similar to the ones in the
map we want to register, one per line (those are printed on the screen
as one navigates through the map in the viewer). A submap of the
registered map can then be extracted, without bundle-adjustment (to
not affect the registration) as:

    $ASTROBEE_WS/devel/lib/sparse_mapping/extract_submap       \
      --skip_bundle_adjustment --input_map registered_map.map  \
      --output_map registered_submap.map                       \
      --skip_bundle_adjustment                                 \
      $(cat list.txt)

If desired, this can be rebuilt to ensure it is created with the same
options as the map we want. That can be done as follows:

    build_map --rebuild --rebuild_detector SURF \
      --output_map registered_submap.map

(This does not affect the camera positions and hence the
registration.)

Our map can be merged into this map without modifying the first map,
and hence keeping its registration, as:

    $ASTROBEE_WS/devel/lib/sparse_mapping/merge_maps \
      --fix_first_map                                \
      --num_image_overlaps_at_endpoints 200          \
      --min_valid_angle 1.0                          \
      registered_submap.map $surf_map                \
      --output_map merged.map

The desired now-registered map can then be extracted as:

    $ASTROBEE_WS/devel/lib/sparse_mapping/extract_submap      \
      --skip_bundle_adjustment                                \
      --input_map merged.map --output_map ${dir}_surf_reg.map \
      ${dir}/*jpg

Here, $dir points to nav_images as earlier in the document.

### Running camera_refiner

Next, the refiner tool is run, as shown below. This will overwrite the
camera calibration file, so it may be prudent to start by copying the
existing calibration file to a new name, and set ASTROBEE_ROBOT to
point to that.

    export ASTROBEE_RESOURCE_DIR=$ASTROBEE_SOURCE_PATH/astrobee/resources
    export ASTROBEE_CONFIG_DIR=$ASTROBEE_SOURCE_PATH/astrobee/config
    export ASTROBEE_WORLD=iss
    export ASTROBEE_ROBOT=bsharp2
    source $ASTROBEE_WS/devel/setup.bash
    source $ISAAC_WS/devel/setup.bash

    float="optical_center focal_length distortion"
    $ISAAC_WS/devel/lib/geometry_mapper/camera_refiner      \
      --ros_bag mybag.bag                                   \
      --sparse_map input_map.map                            \
      --mesh mesh.ply                                       \
      --output_map output_map.map                           \
      --bracket_len 0.6                                     \
      --depth_tri_weight 1000                               \
      --mesh_tri_weight   0                                 \
      --depth_mesh_weight 0                                 \
      --sci_cam_intrinsics_to_float "$float"                \
      --nav_cam_intrinsics_to_float "$float"                \
      --haz_cam_intrinsics_to_float "$float"                \
      --affine_depth_to_image                               \
      --nav_cam_topic /mgt/img_sampler/nav_cam/image_record \
      --num_overlaps 10                                     \
      --out_texture_dir out_texture_dir               

Here it was chosen to pass in a mesh from a previous invocation of the
geometry mapper for this robot and the given registered sparse map (for
example, the ``fused.ply`` mesh can be used). That is optional, and it
is needed only one adds to camera_refiner the constraints that the
triangulated points and haz cam clouds stay close to the mesh, when
positive values should be used for ``--mesh_tri_weight`` and
``--depth_mesh_weight``, and if desired to use the option
``--out_texture_dir``.

Sometimes such a mesh can help with convergence, but should not be
used in a first attempt at calibration. It was not used for the final
bumble robot calibration, when the weights were as above, but was used
for the bsharp2 robot, when all three weights above were set to 25.

Note that sometimes it is desired to not change the nav_cam
intrinsics, such as for the bumble robot, as existing ISS maps depend
on it, and then one should set `--nav_cam_intrinsics_to_float ""`
above.

We used the same bracket length as in the image picker. It is very
important to note that a tight bracket should be used, as, if the
robot moves fast and the bracket value is big, it can affect the
accuracy of calibration. Yet a tight bracket does not allow for wiggle
room if it is decided to vary the timestamp offset (see further down)
while staying within the bounds given by the bracket.

This tool will print some statistics showing the residual errors
before and after each optimization pass (before outlier removal at the
end of the pass), as follows:

    The 25, 50, 75, and 100th percentile residual stats after opt
    depth_mesh_x_m: 0.0015756 0.004064 0.0099792 1.1882 (3566 residuals)
    depth_mesh_y_m: 0.0015745 0.0036734 0.0088576 1.8041 (3566 residuals)
    depth_mesh_z_m: 0.00091998 0.0019874 0.0038565 0.37353 (3566 residuals)
    depth_tri_x_m: 0.00066995 0.0021602 0.0065155 2.7389 (3578 residuals)
    depth_tri_y_m: 0.00069529 0.0022702 0.0069424 2.6967 (3578 residuals)
    depth_tri_z_m: 0.00051406 0.0016069 0.0044358 1.1036 (3578 residuals)
    haz_cam_pix_x: 0.23413 0.51717 1.0606 10353 (3723 residuals)
    haz_cam_pix_y: 0.14605 0.33521 0.67331 1040.7 (3723 residuals)
    mesh_tri_x_m: 0.0030461 0.0097438 0.023452 4.7647 (24231 residuals)
    mesh_tri_y_m: 0.0027831 0.0085864 0.020538 3.1248 (24231 residuals)
    mesh_tri_z_m: 0.00088227 0.0026434 0.0063236 1.1076 (24231 residuals)
    nav_cam_pix_x: 0.055205 0.15801 0.37068 36.164 (206561 residuals)
    nav_cam_pix_y: 0.096944 0.23234 0.49844 495.14 (206561 residuals)
    sci_cam_pix_x: 0.020412 0.10584 0.29013 38.499 (635 residuals)
    sci_cam_pix_y: 0.1585 0.34267 0.71541 30.158 (635 residuals)

These can be helpful in figuring out if the calibration result is good.
The errors whose name ends in "_m" are in meters and measure
the absolute differences between the depth clouds and mesh
(depth_mesh), between depth clouds and triangulated points
(depth_tri), and between mesh points and triangulated points
(mesh_tri), in x, y, and z, respectively. The ``mesh`` residuals will
be printed only if a mesh is passed on input and if the mesh-related
weights are positive. Some outliers are unavoidable, hence some of
these numbers can be big even if the calibration overall does well
(the robust threshold does not allow outliers to dominate).

A source of errors (apart from inaccurate intrinsics, extrinsics, or
insufficiently good modeling of the cameras) can be the
nav_cam_to_sci_cam_timestamp_offset, which can be non-zero if the HLP
and MLP/LLP processors are not synchronized (the sci_cam pictures are
acquired with the HLP and nav_cam with MLP/LLP). If this value is not
known well, this tool can be run with zero or more iterations and
various values of:

    --nav_cam_to_sci_cam_offset_override_value <val>

to see which value gives the smallest residuals. 

If the ``--out_texture_dir`` option is specified, the tool will create
textured meshes for each image and optimized camera at the
end. Ideally those textured meshes will agree among each other.

This program's options are:

    --ros_bag (string, default = "")
      A ROS bag with recorded nav_cam, haz_cam intensity,
      full-resolution sci_cam, and haz_cam depth clouds.

    --sparse_map (string, default = "")
      A registered SURF sparse map made with some of the ROS bag data,
      including nav cam images closely bracketing the sci cam
      images.

    --output_map (string, default = "")
      Output file containing the updated map.

    --nav_cam_topic (string, default = "/mgt/img_sampler/nav_cam/image_record")
      The nav cam topic in the bag file.

    --haz_cam_intensity_topic (string, default = "/hw/depth_haz/extended/amplitude_int")
      The depth camera intensity topic in the bag file.

    --sci_cam_topic (string, default = "/hw/cam_sci/compressed")
      The sci cam topic in the bag file.

    --haz_cam_points_topic (string, default = "/hw/depth_haz/points")
      The depth point cloud topic in the bag file.

    --num_overlaps (int32, default = 10)
      How many images (of all camera types) close and forward in time
      to match to given image.

    --max_haz_cam_image_to_depth_timestamp_diff (double, default = 0.2)
      Use a haz cam depth cloud only if it is within this distance in
      time from a given haz cam intensity image.

    --robust_threshold (double, default = 3.0)
      Residual pixel errors and 3D point residuals (the latter multiplied
      by corresponding weight) much larger than this will be
      exponentially attenuated to affect less the cost function.

    --num_iterations (int32, default = 20)
      How many solver iterations to perform in calibration.

    --bracket_len (double, default = 0.6)
      Lookup sci and haz cam images only between consecutive nav cam
      images whose distance in time is no more than this (in seconds),
      after adjusting for the timestamp offset between these
      cameras. It is assumed the robot moves slowly and uniformly
      during this time. A large value here will make the refiner
      compute a poor solution but a small value will prevent enough
      sci_cam images being bracketed.

    --nav_cam_intrinsics_to_float (string, default = "")
      Refine given nav_cam intrinsics. Specify as a quoted list. For
      example: 'focal_length optical_center distortion'.

    --haz_cam_intrinsics_to_float (string, default = "")
      Refine given haz_cam intrinsics. Specify as a quoted list. For
      example: 'focal_length optical_center distortion'.

    --sci_cam_intrinsics_to_float (string, default = "")
      Refine given sci_cam intrinsics. Specify as a quoted list. For
      example: 'focal_length optical_center distortion'.

    --extrinsics_to_float (string, default = "haz_cam sci_cam depth_to_image")
      Specify the cameras whose extrinsics to float, relative to
      nav_cam. Also consider if to float the haz_cam depth_to_image
      transform.

    --float_scale (bool, false unless specified)
      If to optimize the scale of the clouds, part of
      haz_cam_depth_to_image_transform (use it if the sparse map is
      kept fixed, or else rescaling and registration of the map and
      extrinsics is needed). This parameter should not be used with
      --affine_depth_to_image when the transform is affine, rather
      than rigid and a scale. See also --extrinsics_to_float.

    --float_sparse_map (bool, false unless specified)
      Optimize the sparse map. This should be avoided as it can
      invalidate the scales of the extrinsics and the registration. It
      should at least be used with big mesh weights to attenuate those
      effects. See also: --registration.

    --float_timestamp_offsets (bool, false unless specified)
      If to optimize the timestamp offsets among the cameras.

    --nav_cam_num_exclude_boundary_pixels (int32, default = 0)
      Flag as outliers nav cam pixels closer than this to the
      boundary, and ignore that boundary region when texturing with
      the --out_texture_dir option. This may improve the calibration
      accuracy, especially if switching from fisheye to radtan
      distortion for nav_cam. See also the geometry_mapper
      --undistorted_crop_wins option.

    --timestamp_offsets_max_change (double, default = 1.0)
      If floating the timestamp offsets, do not let them change by
      more than this (measured in seconds). Existing image bracketing
      acts as an additional constraint.

    --nav_cam_to_sci_cam_offset_override_value (double, default = NaN)
      Override the value of nav_cam_to_sci_cam_timestamp_offset from
      the robot config file with this value.

    --depth_tri_weight (double, default = 1000.0)
      The weight to give to the constraint that depth measurements
      agree with triangulated points. Use a bigger number as depth
      errors are usually on the order of 0.01 meters while
      reprojection errors are on the order of 1 pixel.

    --mesh (string, default = "")
      Use this geometry mapper mesh from a previous geometry mapper
      run to help constrain the calibration. E.g., use fused_mesh.ply.

    --mesh_tri_weight (double, default = 0.0)
      A larger value will give more weight to the constraint that
      triangulated points stay close to a preexisting mesh. Not
      suggested by default.

    --depth_mesh_weight (double, default = 0.0)
      A larger value will give more weight to the constraint that the
      depth clouds stay close to the mesh. Not suggested by default.

    --affine_depth_to_image (bool, false unless specified)
      Assume that the depth_to_image_transform for each depth + image
      camera is an arbitrary affine transform rather than a rotation
      times a scale.

    --refiner_num_passes (int32, default = 2)
      How many passes of optimization to do. Outliers will be removed
      after every pass. Each pass will start with the previously
      optimized solution as an initial guess. Mesh intersections (if
      applicable) and ray triangulation will be recomputed before each
      pass.

    --initial_max_reprojection_error (double, default = 300.0)
      If filtering outliers, remove interest points for which the
      reprojection error, in pixels) is larger than this. This
      filtering happens when matches are created, before cameras are
      optimized) and a big value should be used if the initial cameras
      are not trusted.

    --max_reprojection_error (double, default = 25.0)
      If filtering outliers, remove interest points for which the
      reprojection error, in pixels) is larger than this. This
      filtering happens after each optimization pass finishes, unless
      disabled. It is better to not filter too aggressively unless
      confident in the solution.

    --refiner_min_angle (double, default = 0.5)
      If filtering outliers, remove triangulated points for which all
      rays converging to it make an angle (in degrees) less than this.
      Note that some cameras in the rig may be very close to each
      other relative to the triangulated points, so care is needed
      here.

    --refiner_skip_filtering (bool, false unless specified)
      Do not do any outlier filtering.

    --out_texture_dir (string, default = "")
      If non-empty and if an input mesh was provided, project the
      camera images using the optimized poses onto the mesh and write
      the obtained .obj files in the given directory.

    --min_ray_dist (double, default = 0.0)
      The minimum search distance from a starting point along a ray
      when intersecting the ray with a mesh, in meters (if
      applicable).

    --max_ray_dist (double, default = 100.0)
      The maximum search distance from a starting point along a ray
      when intersecting the ray with a mesh, in meters (if
      applicable).

    --nav_cam_distortion_replacement (string, default = "")
      Replace nav_cam's distortion coefficients with this list after
      the initial determination of triangulated points, and then
      continue with distortion optimization. A quoted list of four or
      five values expected, separated by spaces, as the replacement
      distortion model will have radial and tangential
      coefficients. Set a positive
      --nav_cam_num_exclude_boundary_pixels.

    --registration (bool, false unless specified)
      If true, and registration control points for the sparse map
      exist and are specified by --hugin_file and --xyz_file,
      re-register the sparse map at the end. All the extrinsics,
      including depth_to_image_transform, will be scaled accordingly.
      This is not needed if the nav_cam intrinsics and the sparse map
      do not change.

    --hugin_file (string, default = "")
      The path to the hugin .pto file used for sparse map
      registration.

    --xyz_file (string, default = "")
      The path to the xyz file used for sparse map registration.

    --parameter_tolerance (double, default = 1e-12)
      Stop when the optimization variables change by less than this.

    --num_opt_threads (int32, default = 16)
      How many threads to use in the optimization.

    --sci_cam_timestamps (string, default = "")
      Use only these sci cam timestamps. Must be a file with one
      timestamp per line.

    --verbose (bool, false unless specified)
      Print the residuals and save the images and match files. Stereo
      Pipeline's viewer can be used for visualizing these.

### Using the refiner with a radtan model for nav_cam

The camera refiner supports using a radtan distortion model for
nav_cam, that is a model with radial and and tangential distortion,
just like for haz_cam and sci_cam, but the default nav_cam distortion
model is fisheye. One can edit the robot config file and replace the
fisheye model with a desired radial + tangential distortion model (4
or 5 coefficients are needed) then run the refiner.

Since it is not easy to find a good initial set of such coefficients,
the refiner has the option of computing such a model which best fits
the given fisheye model. For that, the refiner is started with the
fisheye model, this model is used to set up the problem, including
triangulating the 3D points after feature detection, then the fisheye
model is replaced on-the-fly with desired 4 or 5 coefficients of the
radtan model via the option --nav_cam_distortion_replacement, to which
one can pass, for example, "0 0 0 0". These coefficients will then be
optimized while keeping the rest of the variables fixed (nav cam focal
length and optical center, intrinsics of other cameras, and all the
extrinsics). The new best-fit distortion model will be written to disk
at the end, replacing the fisheye model, and from then on the new
model can be used for further calibration experiments just as with the
fisheye model.

Since it is expected that fitting such a model is harder at the
periphery, where the distortion is stronger, the camera refiner has
the option ``--nav_cam_num_exclude_boundary_pixels`` can be used to
restrict the nav cam view to a central region of given dimensions when
such such optimization takes place (whether the new model type is fit
on the fly or read from disk when already determined). If a
satisfactory solution is found and it is desired to later use the
geometry mapper with such a model, note its option
``--undistorted_crop_wins``, and one should keep in mind that that the
restricted region specified earlier may not exactly be the region to
be used with the geometry mapper, since the former is specified in
distorted pixels and this one in undistorted pixels.

All this logic was tested and was shown to work in a satisfactory way,
but no thorough attempt was made at validating that a radtan distortion
model, while having more degrees of freedom, would out-perform the
fisheye model. That is rather unlikely, since given sufficiently many
images with good overlap, the effect of the peripheral region where
the fisheye lens distortion may not perform perfectly may be small.

## Orthoprojecting images

Given a camera image, its pose, and a mesh, a useful operation is to
create a textured mesh with this camera. While the geometry mapper can
create textured meshes as well, this tool does so from individual
images rather than fusing them. It uses the logic from the streaming
mapper instead of texrecon which is used by the geometry mapper.

A geometry mapper run directory has all the inputs this tool needs. It
can be run as follows:

    export ASTROBEE_SOURCE_PATH=$HOME/projects/astrobee/src
    export ASTROBEE_WS=$HOME/projects/astrobee
    export ISAAC_WS=$HOME/projects/isaac
    source $ASTROBEE_WS/devel/setup.bash
    source $ISAAC_WS/devel/setup.bash
    export ASTROBEE_RESOURCE_DIR=$ASTROBEE_SOURCE_PATH/astrobee/resources
    export ASTROBEE_CONFIG_DIR=$ASTROBEE_SOURCE_PATH/astrobee/config
    export ASTROBEE_WORLD=iss
    export ASTROBEE_ROBOT=bumble              
    $ISAAC_WS/devel/lib/geometry_mapper/orthoproject                       \
        --camera_name sci_cam                                              \
        --mesh geom_dir/simplified_mesh.ply                                \
        --image geom_dir/distorted_sci_cam/1616785318.1400001.jpg          \
        --camera_to_world geom_dir/1616785318.1400001_sci_cam_to_world.txt \
        --num_exclude_boundary_pixels 0                                    \
        --output_prefix out 

This will write ``out-1616785318.1400001.obj`` and its associated files.

Ensure that the correct robot is specified in ``ASTROBEE_ROBOT``.

Alternatively, the images and cameras can be specified in lists, via
``--image_list`` and ``--camera_list``.
