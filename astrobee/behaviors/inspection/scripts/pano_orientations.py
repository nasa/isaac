#!/usr/bin/env python

"""
This is a reference implementation for how to generate panorama orientations given a
configuration that includes the desired pan and tilt ranges, camera field of view,
desired overlap between consecutive images, and a pre-tilt parameter that helps
with efficiently capturing directional panoramas near tilt = +/- 90.

The main function of interest is panoOrientations(). There is also some
validation logic, and two of the test cases are candidates for the configurations to
test during the SoundSee-Data-3 ISS activity.
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation

EPS = 1e-5
ROLL, PITCH, YAW = 0, 1, 2


def fromRPY(roll, pitch, yaw):
    return Rotation.from_euler("XYZ", (roll, pitch, yaw), degrees=True)


def toRPY(rot):
    return rot.as_euler("XYZ", degrees=True)


def pano1D(rangeMin, rangeMax, fov, overlap):
    """
    Returns image center coordinates such that images cover the range
    @rangeMin .. @rangeMax with at least the specified @overlap. If one
    image suffices, it will be centered between @rangeMin and @rangeMax (and
    the edges of the image will beyond the range if it is smaller than @fov).
    If more than one image is needed to cover the range, the boundary images
    will cover exactly to the edges of the specified range and the images
    will be evenly spaced.

    :param float rangeMin: Minimum angle of minimum image (degrees).
    :param float rangeMax: Maximum angle of maximum image (degrees).
    :param float fov: Field of view of each image (degrees).
    :param float overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
    :return: A vector of orientations of image centers.
    """
    assertLte(rangeMin, rangeMax, EPS)
    W = rangeMax - rangeMin

    if W < fov:
        # Special case: Only one image needed. Center it.
        return np.array([0.5 * (rangeMin + rangeMax)])

    # sufficient overlap criterion: stride <= fov * (1 - overlap)
    # (k - 1) * stride + fov = W
    # stride = (W - fov) / (k - 1)
    # (W - fov) / (k - 1) <= fov * (1 - overlap)
    # k - 1 >= (W - fov) / (fov * (1 - overlap))
    # k >= (W - fov) / (fov * (1 - overlap)) + 1

    k = math.ceil((W - fov) / (fov * (1 - overlap))) + 1

    if 1:
        # optional sanity checks

        stride = (W - fov) / (k - 1)
        assertLte(stride, fov * (1 - overlap), EPS)  # sufficient overlap

        # check if we have more images than necessary
        if k == 1:
            pass  # obviously need at least one image
        elif k == 2:
            assertLte(fov, W, EPS)  # k = 1 is not enough
        else:
            stride1 = (W - fov) / (k - 2)
            assertGte(stride1, fov * (1 - overlap), EPS)  # k is minimized

    minCenter = rangeMin + fov / 2
    maxCenter = rangeMax - fov / 2
    return np.linspace(minCenter, maxCenter, num=k)


def pano1DCompletePan(fov, overlap):
    """
    Returns image center coordinates such that the images cover the full pan
    range -180 .. 180 with at least the specified @overlap between all consecutive images,
    including at the wrap-around, and the image sequence is centered at pan = 0.

    :param float fov: Field of view of each image (degrees).
    :param float overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
    :return: A vector of orientations of image centers.
    """
    k = math.ceil(360 / (fov * (1 - overlap)))
    centers = np.linspace(-180, 180, num=k, endpoint=False)
    # ensure pano is centered at pan = 0
    midPoint = np.mean((centers[0], centers[-1]))
    return centers - midPoint


def panoOrientations(panMin, panMax, tiltMin, tiltMax, hFov, vFov, overlap, preTilt):
    """
    Return image center coordinates that cover the specified pan and tilt ranges,
    with the specified @overlap. Special complete wrap-around behavior is triggered when
    the pan range is exactly -180 .. 180. The @preTilt parameter can be used to re-center
    the panorama, enabling efficient directional panoramas near tilt = +/- 90.

    :param float panMin: Pan angle of left edge of leftmost column (degrees).
    :param float panMax: Pan angle of right edge of rightmost column (degrees).
    :param float tiltMin: Tilt angle of bottom edge of bottom row (degrees).
    :param float tiltMax: Tilt angle of top edge of top row (degrees).
    :param float hFov: Horizontal field of view of each image (degrees).
    :param float vFov: Vertical field of view of each image (degrees).
    :param float overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
    :param float preTilt: Offsets the (pan, tilt) = (0, 0) center in tilt (degrees) so as to efficiently capture panoramas centered near tilt = +/- 90. Example: When preTilt = -90, tilt = 0 is offset to point straight down.
    :return: (imageCenters, ncols, nrows). A list of orientations of image centers, the number of columns in the panorama, and the number of rows.
    """
    if panMin == -180 and panMax == 180:
        panVals = pano1DCompletePan(hFov, overlap)
    else:
        panVals = pano1D(panMin, panMax, hFov, overlap)
    tiltVals = pano1D(tiltMin, tiltMax, vFov, overlap)

    # Images are in column-major order. When capturing a long panorama with
    # people present, this makes it easier for them to move out of the field of
    # view as needed. Order columns left to right, and within each column, tilt
    # top to bottom. That is just more intuitive for people's expectations.
    imageCenters = []
    preRot = fromRPY(0, preTilt, 0)
    for pan in panVals:
        for tilt in reversed(tiltVals):
            rot = fromRPY(0, tilt, pan)
            imageCenters.append(rot * preRot)
    return (imageCenters, len(panVals), len(tiltVals))


def printPano(pano):
    imageCenters, ncols, nrows = pano
    print("%s cols x %s rows [roll pitch yaw]" % (ncols, nrows))
    for i in range(nrows):
        print("  ", end="")
        for j in range(ncols):
            # print(i, j, j * nrows + i)
            imageCenter = imageCenters[j * nrows + i]
            # print(np.around(getEuler(imageCenter, preTilt=-30)), end=" ")
            print(np.around(toRPY(imageCenter)), end=" ")
        print()


def assertEqual(a, b, eps):
    assert abs(a - b) < eps, "FAIL: %s should equal %s, within tolerance %s" % (
        a,
        b,
        eps,
    )


def assertLte(a, b, eps):
    assert a <= b + eps, "FAIL: %s should be <= %s, within tolerance %s" % (a, b, eps)


def assertGte(a, b, eps):
    assert a >= b + eps, "FAIL: %s should be >= %s, within tolerance %s" % (a, b, eps)


def getEuler(rot, preTilt):
    invPreRot = fromRPY(0, -preTilt, 0)
    return toRPY(rot * invPreRot)


def checkPano(pano, panMin, panMax, tiltMin, tiltMax, hFov, vFov, overlap, preTilt):
    imageCenters, ncols, nrows = pano

    topLeft = getEuler(imageCenters[0], preTilt)

    if ncols == 1:
        assertLte(panMax - panMin, hFov, EPS)  # one column is enough
    else:
        nextPan = getEuler(imageCenters[nrows], preTilt)
        panStride = nextPan[YAW] - topLeft[YAW]
        assertLte(panStride, hFov * (1 - overlap), EPS)  # pan overlaps enough

    if nrows == 1:
        assertLte(tiltMax - tiltMin, vFov, EPS)  # one row is enough
    else:
        nextTilt = getEuler(imageCenters[1], preTilt)
        tiltStride = -(nextTilt[PITCH] - topLeft[PITCH])
        assertLte(tiltStride, vFov * (1 - overlap), EPS)  # tilt overlaps enough

    # we shouldn't be able to remove a column
    if ncols == 1:
        pass  # obviously can't remove single column
    elif ncols == 2:
        if panMin == -180 and panMax == 180:
            assertLte(hFov * (1 - overlap), 360)  # one column is not enough
        else:
            assertLte(hFov, panMax - panMin, EPS)  # one column is not enough
    else:
        if panMin == -180 and panMax == 180:
            panStride1 = 360 / (ncols - 1)
        else:
            panStride1 = ((panMax - panMin) - hFov) / (ncols - 2)
        assertGte(panStride1, hFov * (1 - overlap), EPS)  # ncols is minimized

    # we shouldn't be able to remove a row
    if nrows == 1:
        pass  # obviously can't remove single row
    elif nrows == 2:
        assertLte(vFov, tiltMax - tiltMin, EPS)  # one row is not enough
    else:
        tiltStride1 = ((tiltMax - tiltMin) - vFov) / (nrows - 2)
        assertGte(tiltStride1, vFov * (1 - overlap), EPS)  # nrows is minimized

    bottomRight = getEuler(imageCenters[-1], preTilt)

    panCenterMin = topLeft[YAW]
    panCenterMax = bottomRight[YAW]
    if panMin == -180 and panMax == 180:
        assertEqual(panStride * ncols, 360, EPS)  # evenly spaced
        midPan = np.mean((panCenterMin, panCenterMax))
        assertEqual(midPan, 0, EPS)  # centered
    else:
        if ncols == 1:
            assertEqual(
                panCenterMin, 0.5 * (panMin + panMax), EPS
            )  # single column is centered
        else:
            assertEqual(panCenterMin - hFov / 2, panMin, EPS)  # covers to panMin
            assertEqual(panCenterMax + hFov / 2, panMax, EPS)  # covers to panMax

    tiltCenterMax = topLeft[PITCH]
    tiltCenterMin = bottomRight[PITCH]

    if nrows == 1:
        assertEqual(
            tiltCenterMin, 0.5 * (tiltMin + tiltMax), EPS
        )  # single row is centered
    else:
        assertEqual(tiltCenterMin - vFov / 2, tiltMin, EPS)  # covers to tiltMin
        assertEqual(tiltCenterMax + vFov / 2, tiltMax, EPS)  # covers to tiltMax


def testCase(label, config):
    print(label, end=": ")
    pano = panoOrientations(**config)
    printPano(pano)
    checkPano(pano, **config)


# Based on field_of_view_calculator.py, consulting with Oleg, we have:
SCI_CAM_FOV = [60.8, 47.5]  # degrees
HAZ_CAM_FOV = [54.8, 43.2]  # degrees

# For panorama configuration, to ensure all sensors have enough overlap, we
# need to use the combination of (sensor FOV, overlap) that translates to the
# tightest image spacing.

# The justification for requiring overlap is partly in order to use features in
# overlapping areas to guide registration, and partly in order to tolerate
# attitude control errors while collecting the panorama (with more overlap, we
# are less likely to have accidental gaps in coverage due to pointing errors).

# Overlapping for registration is not really relevant for the HazCam, given
# that the geometry mapper currently exclusively relies on NavCam for
# registration, but it is relevant for registering SciCam images using
# stitching tools like Hugin. Currently, we don't really know what level of
# pointing error to expect.

# In the test cases below, we are conservatively using the smaller HazCam FOV
# with the 30% overlap required by the SciCam. Note that if we were to switch
# to basing it on the larger SciCam FOV, it would only reduce the image count
# for a full-sphere pano by ~10-15%, so this seems like an acceptable level
# of overhead in order to be appropriately conservative.

# == TEST CASE 1 ==
# Full spherical panorama. Note a small amount of padding has been added to the
# tilt range to ensure the poles are actually captured. It doesn't change the
# number of rows required.

# These might be the right parameters for a full-coverage SciCam test pano
# during SoundSee-Data-3.
testCase(
    "Full spherical panorama",
    {
        "panMin": -180,
        "panMax": 180,
        "tiltMin": -95,
        "tiltMax": 95,
        "hFov": HAZ_CAM_FOV[0],
        "vFov": HAZ_CAM_FOV[1],
        "overlap": 0.3,
        "preTilt": 0,
    },
)

# == TEST CASE 2 ==
testCase(
    "Directional panorama",
    {
        "panMin": -60,
        "panMax": 60,
        "tiltMin": -30,
        "tiltMax": 30,
        "hFov": HAZ_CAM_FOV[0],
        "vFov": HAZ_CAM_FOV[1],
        "overlap": 0.3,
        "preTilt": 0,
    },
)

# == TEST CASE 3 ==
# This test case with a small amount of pre-tilt is useful for sanity checking.
# We expect preTilt to have a straightforward first-order effect on the pitch
# values but also a second-order effect on the roll and yaw.
testCase(
    "Directional panorama with small pre-tilt",
    {
        "panMin": -60,
        "panMax": 60,
        "tiltMin": -30,
        "tiltMax": 30,
        "hFov": HAZ_CAM_FOV[0],
        "vFov": HAZ_CAM_FOV[1],
        "overlap": 0.3,
        "preTilt": -10,
    },
)

# == TEST CASE 4 ==
# The results look much stranger with pre-tilt set to -90. For example, in the
# center column of the pano, we expect both the roll and yaw values to flip
# from 0 (upper part of pano) to +/- 180 (lower part of pano) as the pano
# crosses the pole. Note: If you see a warning about gimbal lock right at the
# pole, it should be harmless in this context.  It just indicates that there
# are multiple valid RPY values that would give the same desired rotation, so
# it is arbitrarily picking one.

# These might be the right parameters for a SoundSee test pano during
# SoundSee-Data-3. (The FOV parameters are a bit arbitrary for this type of
# "virtual imaging" sensor, can double-check with SoundSee folks.)
testCase(
    "Directional panorama centered straight down",
    {
        "panMin": -60,
        "panMax": 60,
        "tiltMin": -60,
        "tiltMax": 60,
        "hFov": 60,
        "vFov": 60,
        "overlap": 0.3,
        "preTilt": -90,
    },
)

# == TEST CASE 5 ==
# Test special-case logic for 1-row panorama.

testCase(
    "1-row panorama",
    {
        "panMin": -60,
        "panMax": 60,
        "tiltMin": 10,
        "tiltMax": 20,
        "hFov": 60,
        "vFov": 60,
        "overlap": 0.3,
        "preTilt": 0,
    },
)

# == TEST CASE 6 ==
# Test special-case logic for 1-column panorama.

testCase(
    "1-column panorama",
    {
        "panMin": 0,
        "panMax": 0,
        "tiltMin": -60,
        "tiltMax": 60,
        "hFov": 60,
        "vFov": 60,
        "overlap": 0.3,
        "preTilt": 0,
    },
)
