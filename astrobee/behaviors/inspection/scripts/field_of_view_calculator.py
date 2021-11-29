#!/usr/bin/env python

"""
This script documents how to calculate the field of view for some Astrobee
cameras using existing config files. It could easily be expanded to support
other cameras.
"""

import math
from math import atan


def imageFov1(imageSizePixels, focalLengthPixels):
    """
    :param int imageSizePixels: Image size (pixels).
    :param float focalLengthPixels: Focal length (pixels).
    :return: Image field of view (degrees).
    """
    return 2 * atan(imageSizePixels / (2 * focalLengthPixels)) * 180 / math.pi


def imageFov(imageSizePixels, focalLengthPixels):
    return (
        imageFov1(imageSizePixels[0], focalLengthPixels[0]),
        imageFov1(imageSizePixels[1], focalLengthPixels[1]),
    )


# Generic camera config for all robots is at:
#   https://github.com/nasa/astrobee/blob/develop/astrobee/config/cameras.config
# Camera calibration parameters for a specific robot are for example at:
#   https://github.com/nasa/astrobee/blob/develop/astrobee/config/robots/bumble.config
#   (For this purpose, we can just use the known-good Bumble parameters for all robots.)

# To find imageSizePixels, look at:
#   File: cameras.config
#   Field: <camera_name>.{width, height}

# To find focalLengthPixels, look at:
#   File: robots/<robot>.config
#   Field: robot_camera_calibrations.<camera_name>.intrinsic_matrix
#   Entries: The first two diagonal matrix entries [0, 0] and [1, 1].

CONFIGS = [
    {
        "name": "SciCam",
        # Note: SciCam was calibrated using 1/4 resolution images. Full res is 4x bigger.
        "imageSizePixels": [1336, 1002],
        "focalLengthPixels": [1138.4943, 1138.4943],
    },
    {
        "name": "HazCam",
        "imageSizePixels": [224, 171],
        "focalLengthPixels": [215.88697, 215.88697],
    },
]

for config in CONFIGS:
    fov = imageFov(config["imageSizePixels"], config["focalLengthPixels"])
    print("%s FOV: %.1f x %.1f degrees" % (config["name"], fov[0], fov[1]))
