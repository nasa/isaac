#!/usr/bin/env python

"""
This script documents how to calculate the field of view for some Astrobee
cameras using existing config files. It could easily be expanded to support
other cameras.
"""

import math
from math import atan, tan

import scipy.optimize


def tsai_distort(ru, kappa):
    # http://www.vision.caltech.edu/bouguetj/calib_doc/papers/Tsai.pdf eq 6 - Tsai model
    ru2 = ru ** 2
    coeff = 1 + kappa[0] * ru2 + kappa[1] * ru2 ** 2
    # normally it would be ru = rd * coeff, but see camera_params.cc - the sense is inverted
    rd = ru * coeff
    return rd


def tsai_undistort(rd, kappa):
    # solve for ru in: rd = tsai_distort(ru, kappa)
    def func(ru):
        return tsai_distort(ru, kappa) - rd

    ru0 = rd  # initial guess: no distortion
    ru = scipy.optimize.fsolve(func, ru0)
    return ru


def imageFov1(imageSizePixels, focalLengthPixels, omega=None, kappa=None):
    """
    :param int imageSizePixels: Image size (pixels).
    :param float focalLengthPixels: Focal length (pixels).
    :param float omega: Distortion coefficient omega for FOV model.
    :param float kappa: Radial distortion coefficients kappa for Tsai model.
    :return: Image field of view (degrees).
    """
    rd = (imageSizePixels / 2) / focalLengthPixels
    ru = rd
    if omega is not None:
        # https://hal.inria.fr/inria-00267247/document eq. 14 - FOV model
        ru = tan(rd * omega) / (2 * tan(omega / 2))
    elif kappa is not None:
        ru = tsai_undistort(rd, kappa)
    return 2 * atan(ru) * 180 / math.pi


def imageFov(config):
    imageSizePixels = config["imageSizePixels"]
    focalLengthPixels = config["focalLengthPixels"]
    omega = config.get("omega")
    kappa = config.get("kappa")
    return (
        imageFov1(imageSizePixels[0], focalLengthPixels[0], omega, kappa),
        imageFov1(imageSizePixels[1], focalLengthPixels[1], omega, kappa),
    )


# Generic camera config for all robots is at:
#   https://github.com/nasa/astrobee/blob/develop/astrobee/config/cameras.config
# Camera calibration parameters for a specific robot are for example at:
#   https://github.com/nasa/astrobee/blob/develop/astrobee/config/robots/bumble.config
#   (For this purpose, we can just use the known-good Bumble parameters for all robots.)

# To find imageSizePixels, look at:
#   File: cameras.config
#   Field: <camera_name>.{width, height}

# To find focalLengthPixels, omega, kappa, look at:
#   File: robots/<robot>.config
#   focalLengthPixels:
#     Field: robot_camera_calibrations.<camera_name>.intrinsic_matrix
#     Entries: The first two diagonal matrix entries [0, 0] and [1, 1].
#   omega:
#     Field: robot_camera_calibrations.<camera_name>.distortion_coeff
#     Entries: If the value is a scalar, interpret it as omega [FOV model].
#   kappa:
#     Field: robot_camera_calibrations.<camera_name>.distortion_coeff
#     Entries: If the value is a length 4+ vector, interpret entries 0, 1, and 4 (if present) as kappa [Tsai model].

CONFIGS = [
    {
        "name": "SciCam",
        # Note: SciCam was calibrated using 1/4 resolution images. Full res is 4x bigger.
        "imageSizePixels": [1336, 1002],
        "focalLengthPixels": [1138.4943, 1138.4943],
        "kappa": [-0.025598438, 0.0056673533],
    },
    {
        "name": "HazCam",
        "imageSizePixels": [224, 171],
        "focalLengthPixels": [215.88697, 215.88697],
        "kappa": [-0.259498, -0.00024045673],
    },
    {
        "name": "NavCam",
        "imageSizePixels": [1280, 960],
        "focalLengthPixels": [608.8073, 607.61439],
        "omega": 0.998693,
    },
]

for config in CONFIGS:
    fov = imageFov(config)
    print("%s FOV: %.1f x %.1f degrees" % (config["name"], fov[0], fov[1]))
