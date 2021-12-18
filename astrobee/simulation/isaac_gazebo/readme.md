\page sim Simulation

Simulation contains the packages where the dense maps are built


This page describes the isaac gazebo plugins.

# Heat cam

This plugin simulates a heat-detecting camera, with the color in the
images it produces suggestive of the temperature on the surface of the
3D environment seen in the camera.

The file 

  isaac/isaac/config/hw/heat_cam.config

controls the behavior of this camera. It is read only once, when the
simulator is started. Its various parameters as follows:

 - heat_cam_rate: The update rate of the camera. It is suggested to keep
     it at 1.0 for nominal behavior, decrease it to make the camera images
     be updated less often, and to set it to 0.0 to have this sensor off.

 - continuous_picture_taking: If the camera should take pictures
     as often as it can. The alternative is for it to wait until it
     receives a command over the guest science topic when a single
     picture should be taken (see more below).

 - min_radiance and max_radiance: The radiance ("temperature") being
     measured is assumed to fall between these values. Any measurements
     outside of this range will be clamped to it. These values will be
     mapped to the "coolest" and "hottest" colors in the color image.

 - ambient_radiance: The radiance away from any hot or cold spots.

Each hot or cold spot will be assumed to be circular, and will be
characterized by six numbers. They determine a spot's position in
Cartesian coordinates, inner radius of the hot/cold spot, its outer
radius, and its delta radiance.

Inside the inner radius region the radiance will be

  ambient_radiance + delta_radiance

and it will gradually change to the ambient radiance by the time the
boundary of the outer radius is reached.

This camera publishes the RGB heat image, its pose and intrinsics, on topics:

  /hw/cam_heat
  /sim/heat_cam/pose
  /sim/heat_cam/info

To launch the simulator and see the heat map, first set up the environment.
The paths below may need to be adjusted for your system.

  export ASTROBEE_SOURCE_PATH=$HOME/astrobee/src
  export ASTROBEE_BUILD_PATH=$HOME/astrobee
  export ISAAC_WS=$HOME/isaac
  source $ASTROBEE_BUILD_PATH/devel/setup.bash
  source $ISAAC_WS/devel/setup.bash

Then run:

  roslaunch isaac sim.launch world:=iss rviz:=true \
    pose:="11.2 -7.72 5.0 0 0 0 0 1"

Note that as the robot moves and leaves the hot spots behind, the heat
cam will only show the constant ambient temperature.

By default, the camera takes pictures as often as it can. It listens
however to the topic

  /comm/dds/command

for guest science commands that may tell it to take a single picture
at a specific time, or to take pictures continuously. Such a command
must use the app name "gov.nasa.arc.irg.astrobee.heat_cam_image"
(which is the "s" field in the first command argument) for it to be
processed.
