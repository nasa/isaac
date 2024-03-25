\page str_anomaly Image STR

Image STR Anomaly Detection Documentation
====================

Overview
---------

The Image anomaly detector uses Scene Text Recognition (STR) to parse through sci-cam images of the ISS. It creates a database of all the 
labels found on the ISS and displays the results of a search label using the panorama interface https://ivr.ndc.nasa.gov/isaac_panos/.


Prerequisites 
---------

This package relies on the CRAFT-Pytorch and PARSeq libraries. To ensure the Image STR package works smoothly, clone the repos associated with the 
libraries and build them as Python packages.

CRAFT-Pytorch

    pip install https://github.com/marinagmoreira/CRAFT-pytorch.git#egg=craft

PARSeq

	pip install https://github.com/marinagmoreira/parseq.git@focal#egg=parseq

The Image STR package also contains a requirements file with all the other necessary Python packages. 

	pip install -r requirements.txt

Running the Code
---------

The python code containing the label detection and search is in parse_img.py.

Parameters
```
--bag_path BAG_PATH   Path to bag folder where the images came from.
--image_file IMAGE_FILE
                    Path to image to parse.
--image_folder IMAGE_FOLDER
                    Path to image folder to parse images.
--result_folder RESULT_FOLDER
                    Path to result folder to save results.
--increment INCREMENT
                    If True, will save the results of each individual
                    image.
--df_file DF_FILE   If provided, will create an ocr using data from csv
                    file.
```

To see the demo, run the Analyst Notebook. Directions are specified in the readme.md in the Analyst Folder.