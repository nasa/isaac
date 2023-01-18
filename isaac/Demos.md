\page demos_native Demos

To run demos using docker containers, please see \ref docker. There you'll find instructions on how to run the containers and available demos.


# Starting ISAAC FSW

    roslaunch isaac sim.launch dds:=false robot:=sim_pub rviz:=true

# Native Demos

## Inspection Demos

The inspection node facilitates the robot to take inspect its surroundings, there are multiple modes to do so. If the robot is not already undocked, it will do so when the inspection command is executed.
There are many costuymization options available to the inspection tool, so please check the help output with:

    rosrun inspection inspection_tool -help

### Anomaly

Used to take a close-up picture of an area and analyses it with the image anomaly detection node:

    rosrun inspection inspection_tool -anomaly

The robot will inspect the target defined in astrobee/behaviors/inspection/resources/inspection_iss.txt by default, which is a vent on entry of the JEM, bay1. The robot will generate the survey, go to the inspection point and take a picture with the sci camera. The incoming picture will be analysed by the image anomaly detector. In this case it will report back whether the analysed vent is free or obstructed.
Note: if the image anomaly detector was not launched with the fsw, then it will only take the picture and skip the analysis.

Options include: target_distance (desired distance to target); target_size_x (target size x - width); target_size_y (target size y - height)


### Geometry

Used to create a geometric model of an area (3D model with texture). Takes pictures at all the locations specified in the survey plan.

    rosrun inspection inspection_tool -geometry

The robot will inspect the target defined in astrobee/behaviors/inspection/resources/geometry_iss.txt by default, which corresponds to the bay 5 in the JEM. The robot will go to all locations, and after stable stationkeep, will take a sci camera image. When the image is confirmed to have been received the robot moves forward to another station.

For instructions on how to analysed the abtained data recorded in a bagfile, go to \ref geometric_streaming_mapper.


### Volumetric

Used to create a volumetric model of a given signal.

    rosrun inspection inspection_tool -volumetric

The robot will inspect the target defined in astrobee/behaviors/inspection/resources/volumetric_iss.txt by default, which corresponds to going around the JEM module. The robot stops at each station and then continue to the next.

To learn more about how to process this data, consult \ref volumetric_mapper. Data types that can be scoped though this method are signals such as wifi signal strength and RFID tags.

### Panorama

Used to take pictures of a certain location that can be stitched into a panorama.

    rosrun inspection inspection_tool -panorama

The robot will take pictures with the camera centered at the location defined in the survey file in astrobee/behaviors/inspection/resources/panorama_iss.txt. The inspection node generates the survey given the parameters provided or derived from the camera model, therefore the pose specified in the survey file is the panorama center and not each station coordinate. The robot will take pictures at each generated station similarly to the geometry mode.

Options include: h_fov (camera horizontal fov, default -1 uses camera matrix); max_angle (maximum angle (deg) to target); max_distance (maximum distance to target); min_distance (minimum distance to target); overlap (overlap between images); pan_max (maximum pan); pan_min (minimum pan).


## Cargo Transport

In simulation, it is possible to perform cargo transfer using Astrobee. To do so you will tave to spawn the cargo at a certain location, and send the commands to pick up and drop the cargo.

To spawn a cargo:

    roslaunch isaac_gazebo spawn_object.launch spawn:=cargo pose:="11.3 -5.6 5.7 -0.707 0 0 0.707" name:=CTB_05_1070

To pick up the cargo (make sure it's undocked) - the pose is the cargo pose:

    rosrun cargo cargo_tool -id CTB_05_1070 -pick -pose "11.3 -5.6 5.7 -0.707 0 0 0.707"

To drop cargo - the pose is the dock pose:

    rosrun cargo cargo_tool -drop -pose "11.3 -5.6 5.955 -0.707 0 0 0.707"