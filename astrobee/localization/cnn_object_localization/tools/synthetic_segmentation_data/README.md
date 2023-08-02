## To run data generation:

1. Check to make sure that no instances of Ignition Gazebo are already running. Multiple instances WILL prevent data from being generated. Use `top` and look for instances of `ruby`.
2. Build the contents of `ign_plugins`. (Ensure that `ign_plugins/build/libDataGeneration.so` exists.)
3. `./generate_data.sh -c CONFIG_NAME -o PATH/TO/OUTPUT/DIR`, e.g. `./generate_data.sh -c handrail -o data`
4. Click the refresh button in Image Display, then select one of the two "/segmentation_camera/..." topics. (This causes the ignition gazebo GUI to subscribe to the sensor, which forces it to run continuously. Otherwise, it won't generate any data.)
5. Click the run button.
