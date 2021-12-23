\page acoustics_camera Acoustics camera

### Overview

This camera simulates a microphone array, or, in other words, a
directional microphone. Its readings are assembled into a spherical
pattern, consisting of one floating-point measurement for each
direction emerging from the microphone center. It is assumed that the
microphone array is mounted on the robot and it takes readings as the
robot moves around.

For visualization purposes, the microphone measurements are converted
to an acoustic "image". Hence, a virtual camera is created centered at
the microphone and with a certain pose that is ideally facing the
direction where all or most of the interesting sounds are coming
from. The reading at a pixel of that camera is the value of the
microphone measurement in the direction of the ray going from the
microphone (and camera) center through that pixel.

### Installation

The acoustics camera depends on the pyroomacoustics package. This
package can be installed together with its dependencies in a Python
2.7 environment using the command:

    pip install numpy==1.15.4 scipy==0.18 pillow==6 PyWavelets==0.4.0 \
      networkx==1.8 matplotlib==2.0.0 scikit-image==0.14              \
      pyroomacoustics==0.3.1

It would normally install itself in:

    $HOME/.local/lib/python2.7/site-packages/pyroomacoustics

### Running the acoustics camera

The acoustics camera ROS node can be run as part of the simulator. For that,
first set up the environment along the lines of:

    export ASTROBEE_SOURCE_PATH=$HOME/astrobee/src
    export ASTROBEE_BUILD_PATH=$HOME/astrobee
    export ISAAC_WS=$HOME/isaac
    source $ASTROBEE_BUILD_PATH/devel/setup.bash
    source $ISAAC_WS/devel/setup.bash

then run:

    roslaunch isaac sim.launch world:=iss rviz:=true acoustics_cam:=true \
      pose:="11.2 -7.72 5.0 0 0 0 0 1"

One must ensure that the "DEBUG: AcousticsCam" image checkbox is
checked in RViz upon launch, to be able to visualize this image. If it
is not checked, it should be checked manually, the current config
should be saved from the RViz menu, and the simulation should be
restarted.

It is also possible to start the simulator without this camera,
and then launch this camera separately as:

    source $ASTROBEE_BUILD_PATH/devel/setup.bash
    source $ISAAC_WS/devel/setup.bash
    roslaunch acoustics_cam acoustics_cam.launch output:=screen

The acoustics camera can be run without ROS as:

    $ISAAC_WS/src/astrobee/simulation/acoustics_cam/nodes/acoustics_cam debug_mode

In that case it assumes that the robot pose is the value set in the
field "debug_robot_pose" in acoustics_cam.json (see below). In this
mode it will only create a plot of the acoustics cam image. The
sources of sounds will be represented as crosses in this plot, and the
camera (microphone) position will be shown as a star.

### ROS communication

The acoustics camera subscribes to

    /loc/truth/pose

to get the robot pose. It publishes its image, camera pose, and camera
intrinsics on topics:

    /hw/cam_acoustics
    /sim/acoustics_cam/pose
    /sim/acoustics_cam/info

By default, the camera takes pictures as often as it can (see the
configuration below), which is rarely, in fact, as it is slow. It
listens however to the topic 

    /comm/dds/command

for guest science commands that may tell it to take a single picture
at a specific time, or to take pictures continuously. Such a command
must use the app name "gov.nasa.arc.irg.astrobee.acoustics_cam_image"
(which is the "s" field in the first command argument) for it to be
processed.

### Configuration

The behavior of this camera is described in 

     $ISAAC_WS/src/astrobee/simulation/acoustics_cam/acoustics_cam.json

It has the following entries:

- room_corners: the min and max coordinates of the simulated room in
  which the directional microphone takes measurements. Since the room
  is assumed to be anechoic, so there is no reflection from room walls,
  the precise room dimensions are not important, as long as the room is
  big enough to contain the sources of sound and the possible positions
  of the microphone and the robot carrying it.
  
- sound_sources: the locations of the sound sources, their strengths,
  and files having the specific sound patterns. Those can be used,
  together with a machine learning algorithm, to differentiate among
  different types of sound.

- camera_to_body_transform: The transform from the camera to the robot
  body coordinate system.

- distance_between_updates: How far the microphone/robot should move,
  in meters, before another reading can take place. Set this to 0 to 
  remove this constraint.
 
- continuous_picture_taking: If the camera should take pictures as often 
  as it can (value is 1). The alternative is for it to wait until
  it receives a command over the guest science topic when a single
  picture should be taken (value is 0).

- image_width, image_height, hfov: The camera image width, image
  height, and horizontal field of view (in radians).

- intensity_range: The reconstructed sound intensity as measured by
  the acoustic camera will be clamped to this range and then scaled to
  the range of 0 to 255, and rounded to integer. The obtained grayscale
  image will be colorized and published.

- debug_robot_pose: This is useful for testing this node without ROS.
  The translation and quaternion fields specified here correspond to 
  what is published by the /loc/truth/pose topic.

