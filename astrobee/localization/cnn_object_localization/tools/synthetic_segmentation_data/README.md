### To run data generation:

0. Check to make sure that no instances of Ignition Gazebo are already running. Multiple instances WILL prevent data from being generated.
1. Build the contents of `ign_plugins`. (Ensure that `ign_plugins/build/libDataGeneration.so` exists.)
2. `./generate_data.sh -c CONFIG_NAME -o PATH/TO/OUTPUT/DIR`, e.g. `./generate_data.sh -c handrail -o data`
3. Click the refresh button in Image Display, then select one of the two "/segmentation_camera/..." topics. (This causes the ignition gazebo GUI to subscribe to the sensor, which forces it to run continuously. Otherwise, it won't generate any data.)
4. Click the run button.

