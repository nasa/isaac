#!/usr/bin/env python

"""
This is a reference implementation for how to generate panorama orientations given a
configuration that includes the desired pan and tilt ranges, camera field of view,
desired overlap between consecutive images, and attitude tolerance to account
for the possibility of pointing error.

The main function of interest is panoOrientations(). There is also some
validation logic, and two of the test cases are candidates for the configurations to
test during ISAAC ISS activities.
"""

import math

import numpy as np

EPS = 1e-5


def getHFovEffective(hFov, vFov, tilt):
    """
    Returns the effective HFOV for an image with tilt centered at @tilt.
    At large tilt values ("high latitude"), effective HFOV is larger
    because the parallels are shorter near the pole. We conservatively
    take the effective HFOV from either the middle, top, or bottom edge
    of the image, whichever is smallest.

    :param float hFov: Horizontal field of view of each image (degrees).
    :param float vFov: Vertical field of view of each image (degrees).
    :param float tilt: The tilt of the image center (degrees).
    :return: The effective HFOV (degrees).
    """
    vRadius = 0.5 * vFov
    minTheta = min(abs(tilt - vRadius), abs(tilt), abs(tilt + vRadius))
    return hFov / math.cos(minTheta * math.pi / 180)


def pano1D(rangeRadius, fov, overlap, attitudeTolerance):
    """
    Returns image center coordinates such that images cover the range
    -@rangeRadius .. +@rangeRadius with at least the specified
    @overlap. If one image suffices, it will be centered at 0 (and the
    edges of the image will extend beyond the range if it is smaller
    than @fov).  If more than one image is needed to cover the range,
    the boundary images will cover exactly to the edges of the specified
    range (modulo @attitudeTolerance) and the images will be evenly spaced.

    :param float rangeRadius: Images must cover -rangeRadius .. +rangeRadius (degrees).
    :param float fov: Field of view of each image (degrees).
    :param float overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
    :param float attitudeTolerance: Ensure overlap criterion is met even if relative attitude between a pair of adjacent images, or between an image and the desired pano boundary, is off by at most this much (degrees).
    :return: A vector of orientations of image centers.
    """
    W = rangeRadius * 2

    if W < fov:
        # Special case: Only one image needed. Center it.
        return np.array([0])

    # sufficient overlap criterion: stride <= fov * (1 - overlap) - attitudeTolerance
    # (k - 1) * stride + fov = W + 2 * attitudeTolerance
    # stride = (W + 2 * attitudeTolerance - fov) / (k - 1)
    # (W + 2 * attitudeTolerance - fov) / (k - 1) <= fov * (1 - overlap) - attitudeTolerance
    # k - 1 >= (W + 2 * attitudeTolerance - fov) / (fov * (1 - overlap) - attitudeTolerance)
    # k >= (W + 2 * attitudeTolerance - fov) / (fov * (1 - overlap) - attitudeTolerance) + 1

    k = (
        math.ceil(
            (W + 2 * attitudeTolerance - fov)
            / (fov * (1 - overlap) - attitudeTolerance)
        )
        + 1
    )

    if 1:
        # optional sanity checks

        stride = (W + 2 * attitudeTolerance - fov) / (k - 1)
        assertLte(
            stride, fov * (1 - overlap) - attitudeTolerance, EPS
        )  # sufficient overlap

        # check if we have more images than necessary
        if k == 1:
            pass  # obviously need at least one image
        elif k == 2:
            assertLte(fov, W + 2 * attitudeTolerance, EPS)  # k = 1 is not enough
        else:
            stride1 = (W + 2 * attitudeTolerance - fov) / (k - 2)
            assertGte(
                stride1, fov * (1 - overlap) - attitudeTolerance, EPS
            )  # k is minimized

    minCenter = -(rangeRadius + attitudeTolerance) + fov / 2
    maxCenter = -minCenter
    return np.linspace(minCenter, maxCenter, num=k)


def pano1DCompletePan(fov, overlap, attitudeTolerance):
    """
    Returns image center coordinates such that the images cover the full pan
    range -180 .. 180 with at least the specified @overlap between all consecutive images,
    including at the wrap-around, and the image sequence is centered at pan = 0.

    :param float fov: Field of view of each image (degrees).
    :param float overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
    :param float attitudeTolerance: Ensure overlap criterion is met even if relative attitude between a pair of adjacent images, or between an image and the desired pano boundary, is off by at most this much (degrees).
    :return: A vector of orientations of image centers (degrees).
    """
    k = math.ceil(360 / (fov * (1 - overlap) - attitudeTolerance))
    centers = np.linspace(-180, 180, num=k, endpoint=False)
    # ensure pano is centered at pan = 0
    midPoint = np.mean((centers[0], centers[-1]))
    return centers - midPoint


def pano1DPan(panRadius, hFov, vFov, overlap, attitudeTolerance, tilt):
    """
    Returns image center coordinates for a row of images at tilt value
    @tilt that cover the range -@panRadius .. +@panRadius with at least
    the specified @overlap. Special complete wrap-around behavior is
    triggered when @panRadius is exactly 180.

    :param float panRadius: Cover pan angle of -panRadius to +panRadius (degrees).
    :param float hFov: Horizontal field of view of each image (degrees).
    :param float vFov: Vertical field of view of each image (degrees).
    :param float overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
    :param float attitudeTolerance: Ensure overlap criterion is met even if relative attitude between a pair of adjacent images is off by at most this much (degrees).
    :param float tilt: The tilt of the image center (degrees).
    :return: A vector of pan orientations of image centers (degrees).
    """
    hFovEffective = getHFovEffective(hFov, vFov, tilt)
    # print("tilt=%s hFovEffective=%s" % (tilt, hFovEffective))
    if panRadius == 180:
        return pano1DCompletePan(hFovEffective, overlap, attitudeTolerance)
    else:
        return pano1D(panRadius, hFovEffective, overlap, attitudeTolerance)


def panoOrientations(panRadius, tiltRadius, hFov, vFov, overlap, attitudeTolerance):
    """
    Return image center coordinates that cover the specified pan and tilt ranges,
    with the specified @overlap. Special complete wrap-around behavior is triggered when
    the panRadius is exactly 180.

    :param float panRadius: Cover pan angle of -panRadius to +panRadius (degrees).
    :param float tiltMin: Cover tilt angle of -tiltRadius to +tiltRadius (degrees).
    :param float hFov: Horizontal field of view of each image (degrees).
    :param float vFov: Vertical field of view of each image (degrees).
    :param float overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
    :param float attitudeTolerance: Ensure overlap criterion is met even if relative attitude between a pair of adjacent images is off by at most this much (degrees).
    :return: (imageCenters, ncols, nrows). A list of orientations of image centers, the number of columns in the panorama, and the number of rows.
    """

    # calculate all image centers
    imageCenters = []
    tiltVals = pano1D(tiltRadius, vFov, overlap, attitudeTolerance)
    for iy, tilt in enumerate(reversed(tiltVals)):
        panVals = pano1DPan(panRadius, hFov, vFov, overlap, attitudeTolerance, tilt)
        for pan in panVals:
            imageCenters.append((pan, tilt, iy, -1))
    # imageCenters = np.array(imageCenters)
    # print(imageCenters.shape)
    imageCenters = np.array(
        imageCenters,
        dtype=[
            ("pan", np.double),
            ("tilt", np.double),
            ("iy", np.int16),
            ("ix", np.int16),
        ],
    )

    # assign image centers to columns
    minTilt = np.min(np.abs(tiltVals))
    columnCenters = pano1DPan(
        panRadius, hFov, vFov, overlap, attitudeTolerance, minTilt
    )
    imagePans = imageCenters["pan"]
    imageCenters["ix"] = np.array(
        [np.argmin(np.abs(columnCenters - p)) for p in imagePans]
    )

    # order images in column-major order; alternate direction top-to-bottom or bottom-to-top
    imagesOrdered = []
    for ix, pan in enumerate(columnCenters):
        imagesInColInd = imageCenters["ix"] == ix
        imagesInCol = imageCenters[imagesInColInd]

        # sort on tilt bottom-to-top
        imagesInCol = imagesInCol[np.argsort(imagesInCol["tilt"])]

        # on even-numbered columns, reverse tilt order, top-to-bottom
        if ix % 2 == 0:
            imagesInCol = imagesInCol[::-1]

        imagesOrdered += list(imagesInCol)
    imagesOrdered = np.array(imagesOrdered)

    ncols = len(columnCenters)
    nrows = len(tiltVals)

    return imagesOrdered, ncols, nrows


def printPano(pano):
    imageCenters, ncols, nrows = pano
    numImages = imageCenters.shape[0]
    print(
        "%s images, %s cols x %s rows, frame# [pan tilt]:" % (numImages, ncols, nrows)
    )
    if 0:
        print(imageCenters)
        return
    imageLookup = {
        (iy, ix): (i, pan, tilt) for i, (pan, tilt, iy, ix) in enumerate(imageCenters)
    }
    for iy in range(nrows):
        print("  ", end="")
        for ix in range(ncols):
            imageInfo = imageLookup.get((iy, ix))
            if imageInfo is None:
                print("              ", end="   ")
            else:
                i, pan, tilt = imageInfo
                print("%2d [%4d %4d]" % (i, round(pan), round(tilt)), end="   ")
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


def testCase(label, config):
    print(label, end=": ")
    pano = panoOrientations(**config)
    printPano(pano)
    print()


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
# Full spherical panorama.

# These might be the right parameters for a full-coverage SciCam test pano.
testCase(
    "Full spherical panorama",
    {
        "panRadius": 180,
        "tiltRadius": 90,
        "hFov": HAZ_CAM_FOV[0],
        "vFov": HAZ_CAM_FOV[1],
        "overlap": 0.3,
        "attitudeTolerance": 5,
    },
)

# == TEST CASE 2 ==
testCase(
    "Directional panorama",
    {
        "panRadius": 60,
        "tiltRadius": 30,
        "hFov": HAZ_CAM_FOV[0],
        "vFov": HAZ_CAM_FOV[1],
        "overlap": 0.3,
        "attitudeTolerance": 5,
    },
)

# == TEST CASE 3 ==
# Test special-case logic for 1-row panorama.

testCase(
    "1-row panorama",
    {
        "panRadius": 60,
        "tiltRadius": 5,
        "hFov": 60,
        "vFov": 60,
        "overlap": 0.3,
        "attitudeTolerance": 5,
    },
)

# == TEST CASE 4 ==
# Test special-case logic for 1-column panorama.

testCase(
    "1-column panorama",
    {
        "panRadius": 0,
        "tiltRadius": 60,
        "hFov": 60,
        "vFov": 60,
        "overlap": 0.3,
        "attitudeTolerance": 5,
    },
)
