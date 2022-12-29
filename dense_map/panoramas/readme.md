\page panoramas Panoramas

Using the stitch panorama script
---------

This script firstly creates the hugin panorama stitcher file for a specific panorama, initializes it with the localization data, runs multiple optimizations and finally produces the final stitched result

To run the tool:

	rosrun inspection stitch_panorama.py -bag_name $BAG_NAME -work_dir $SCI_CAM_DIR

where $BAG_NAME is the bagfile which the script uses to identify which sci_cam images to use and initializes the attitude in the hugin file accordig to localization data. The $SCI_CAM_DIR is the folder location to the sci_cam images copied from the HLP.

Other optional arguments are:

    -input_hugin     (Input Hugin pto file.) type: string
    -output_hugin    (Output Hugin pto file.) type: string
    --no-stitching   (Generate hugin and optimize only.)
    --stitching-only (Stitch hugin file only)

The --no-stitching option followed by the --stitch-only option is useful if manual checks/changes are desired to the hugin file before stitching the panorama (which takes a long time), it particularly useful if the localization data is not sufficient for the panorama output to be properly centered around the (0,0,0) ISS reference frame.  