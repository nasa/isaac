# Overview:

`generate_data.sh` generates synthetic segmentation data using Ignition Gazebo. The output consists of images and corresponding segmentation masks (they are technically instance segmentation masks, but the distinction between instance and semantic segmentation is irrelevant here because exactly one instance appears per image) for ISS handrails. There are a total of 24 handrails in known positions, and the camera takes images from random positions in front of each handrail. A ground truth CSV file is also generated, with one row per output image containing information about the handrail's class (corresponding to its length) as well as the poses of the handrail and camera. 

`generate_data.sh` generates data using a combination of two Ignition Gazebo plugins. One is a built-in instance segmentation camera plugin, and one is a custom DataGeneration plugin which randomizes the camera's pose. Each handrail corresponds to a user-configurable "inspection pose" which looks at the handrail dead-on; the DataGeneration plugin randomly selects poses centered around this "inspection pose". The DataGeneration plugin also keeps track of the number of random poses it has been through for each handrail, and moves on to the next handrail when enough data has been collected The segmentation camera and DataGeneration plugins do not talk to each other at all; instead, the instance segmentation camera plugin is configured to collect data at exactly 1Hz, and the DataGeneration plugin moves the camera every second, thus ensuring that things are syncronized between the two.

`generate_data.sh` is not very generalizable and currently it would take substantial work to modify it to collect data for anything other than handrails. The primary reason for this is that it uses a custom .world file that contains 24 modified versions of the 3D ISS model, each with a handrail in a different location. To generate data for some other 3D object, one would need to modify the .world file by manually placing these 3D objects at various poses and configuring corresponding poses that the camera should record from. It would be worth looking into ways to replace this component of the data generation process with something that offers more flexibility. It is also worth noting that, since we have switched to a keypoint-based approach, it is no longer necessary to generate segmentation mask data. The primary motivation for using Ignition Gazebo was the availability of the built-in segmentation camera plugin, so now that this is no longer necessary, it may be worth looking into using other methods for synthetic data generation.

`annotate_keypoints.sh` is a downstream step that adds additional information to the output of `generate_data.sh`. It provides information about keypoints in the synthetic dataset; in the case of handrails, this means the location of the two ends of each handrail. Given 3D keypoint locations relative to the handrail, it uses the ground truth pose information output by `generate_data.sh` to annotate keypoints in images, generate keypoint masks, and generate keypoint ground truth data.

`annotate_keypoints.sh` is more generalizable than `generate_data.sh`. It can be configured to support arbitrary different classes of objects, with each class of object having its own set of keypoint positions and classes.

## To run data generation:

1. Check to make sure that no instances of Ignition Gazebo are already running. Multiple instances WILL prevent data from being generated. Use `top` and look for instances of `ruby`.
2. Build the contents of `ign_plugins`. (Ensure that `ign_plugins/build/libDataGeneration.so` exists.)
3. `./generate_data.sh -c CONFIG_NAME -o $OUTPUT_DIR`, e.g. `./generate_data.sh -c handrail -o data`
4. Click the refresh button in Image Display, then select one of the two "/segmentation_camera/..." topics from the dropdown menu. (This causes the ignition gazebo GUI to subscribe to the sensor, which forces it to run continuously. Otherwise, it won't generate any data.)
5. Click the run button. Once all target objects are done, CTRL+C from the terminal to exit.

NOTE: Due to a weird off-by-one bug that I can't quite eradicate, the first generated set of images (`$OUTPUT_DIR/colored_maps/colored_0000000.png`, `$OUTPUT_DIR/images/image_0000000.png`, and `$OUTPUT_DIR/labels_maps/labels_0000000.png`) do not have ground truth data recorded; nor do they get counted towards `NUM_IMAGES_EACH`. The easiest way to deal with this issue is to just delete these three images after data generation. `generate_data.sh` has an interrupt hook that should handle this, so you probably don't need to do anything assuming Ignition Gazebo quits gracefully (not a foregone conclusion), but be aware.

## To run data generation:

1. `./annotate_keypoints -c CONFIG_NAME -o $OUTPUT_DIR [-a] [-m] [-j]`, e.g. `./annotate_keypoints -c handrail -o data [-a] [-m] [-j]`. The three flags correspond, respectively, to generating keypoint-annotated images (useful for debug/visualization), generating keypoint masks (useful for training networks that expect mask targets) and generating a ground truth json file (useful for networks that expect 2D point or bounding box targets).

## Notes on config files:

Both `generate_data.sh` and `annotate_keypoints.sh` use a shared config file located in the `./config` folder.

- The format of `KEYPOINTS` is as follows, where `<OBJECT_CLASS>` and `<KEYPOINT_CLASS>` must be positive integers. Unfortunately, everything has to be formatted on a single line. Sorry about that.
    ```
    {
        "<OBJECT_CLASS>":
        [
            {"position": [<X>, <Y>, <Z>], "class": <KEYPOINT_CLASS>}, 
            {"position": [<X>, <Y>, <Z>], "class": <KEYPOINT_CLASS>},
            ...
        ],
        "<OBJECT_CLASS>":
        [
            {"position": [<X>, <Y>, <Z>], "class": <KEYPOINT_CLASS>}, 
            {"position": [<X>, <Y>, <Z>], "class": <KEYPOINT_CLASS>},
            ...
        ]
        ...
    }
    ```
