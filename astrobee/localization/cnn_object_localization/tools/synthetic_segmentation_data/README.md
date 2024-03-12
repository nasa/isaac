## To run data generation:

1. Check to make sure that no instances of Ignition Gazebo are already running. Multiple instances WILL prevent data from being generated. Use `top` and look for instances of `ruby`.
2. Build the contents of `ign_plugins`. (Ensure that `ign_plugins/build/libDataGeneration.so` exists.)
3. `./generate_data.sh -c CONFIG_NAME -o $OUTPUT_DIR`, e.g. `./generate_data.sh -c handrail -o data`
4. Click the refresh button in Image Display, then select one of the two "/segmentation_camera/..." topics from the dropdown menu. (This causes the ignition gazebo GUI to subscribe to the sensor, which forces it to run continuously. Otherwise, it won't generate any data.)
5. Click the run button. Once all target objects are done, CTRL+C from the terminal to exit.

NOTE: Due to a weird off-by-one bug that I can't quite eradicate, the first generated set of images (`$OUTPUT_DIR/colored_maps/colored_0000000.png`, `$OUTPUT_DIR/images/image_0000000.png`, and `$OUTPUT_DIR/labels_maps/labels_0000000.png`) do not have ground truth data recorded; nor do they get counted towards `NUM_IMAGES_EACH`. The easiest way to deal with this issue is to just delete these three images after data generation. `generate_data.sh` has an interrupt hook that should handle this, so you probably don't need to do anything assuming Ignition Gazebo quits gracefully (not a foregone conclusion), but be aware.

## To run keypoint annotation (EXPERIMENTAL):

- `./annotate_keypoints.sh -c CONFIG_NAME -o $OUTPUT_DIR`

Note: If you're running the `../python_rcnn_training/src/*_kprcnn.py` scripts, you want to generate data using this tool.
However, if you're working with the newer models in the `lightweight_cnn_object_localization` package, use the data generation tool there, as there have been some updates.

