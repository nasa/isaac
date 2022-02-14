#!/usr/bin/env python

"""
This is a reference implementation for how to generate panorama orientations given a
configuration that includes the desired pan and tilt ranges, camera field of view,
desired overlap between consecutive images, and attitude tolerance to account
for the possibility of pointing error.

The main function of interest is panoOrientations(). There is also some
validation logic, and some of the test cases are candidates for the configurations to
test during ISAAC ISS activities.
"""

from __future__ import print_function

import argparse
import csv
import logging
import math

import numpy as np

EPS = 1e-5


def get_h_fov_effective(h_fov, v_fov, tilt):
    """
    Returns the effective HFOV for an image with tilt centered at @tilt.
    At large tilt values ("high latitude"), effective HFOV is larger
    because the parallels are shorter near the pole. We conservatively
    take the effective HFOV from either the middle, top, or bottom edge
    of the image, whichever is smallest.

    :param float h_fov: Horizontal field of view of each image (degrees).
    :param float v_fov: Vertical field of view of each image (degrees).
    :param float tilt: The tilt of the image center (degrees).
    :return: The effective HFOV (degrees).
    """
    v_radius = 0.5 * v_fov
    min_abs_theta = min(abs(tilt - v_radius), abs(tilt), abs(tilt + v_radius))
    return h_fov / math.cos(min_abs_theta * math.pi / 180)


def pano_1d(range_radius, fov, overlap, attitude_tolerance):
    """
    Returns image center coordinates such that images cover the range
    -@range_radius .. +@range_radius with at least the specified
    @overlap. If one image suffices, it will be centered at 0 (and the
    edges of the image will extend beyond the range if it is smaller
    than @fov).  If more than one image is needed to cover the range,
    the boundary images will cover exactly to the edges of the specified
    range (modulo @attitude_tolerance) and the images will be evenly spaced.

    :param float range_radius: Images must cover -range_radius .. +range_radius (degrees).
    :param float fov: Field of view of each image (degrees).
    :param float overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
    :param float attitude_tolerance: Ensure overlap criterion is met even if relative attitude between a pair of adjacent images, or between an image and the desired pano boundary, is off by at most this much (degrees).
    :return: A vector of orientations of image centers.
    """
    W = range_radius * 2

    if (W + 2 * attitude_tolerance - fov) < 0:
        # Special case: Only one image needed. Center it.
        return np.array([0])

    # sufficient overlap criterion: stride <= fov * (1 - overlap) - attitude_tolerance
    # (k - 1) * stride + fov = W + 2 * attitude_tolerance
    # stride = (W + 2 * attitude_tolerance - fov) / (k - 1)
    # (W + 2 * attitude_tolerance - fov) / (k - 1) <= fov * (1 - overlap) - attitude_tolerance
    # k - 1 >= (W + 2 * attitude_tolerance - fov) / (fov * (1 - overlap) - attitude_tolerance)
    # k >= (W + 2 * attitude_tolerance - fov) / (fov * (1 - overlap) - attitude_tolerance) + 1

    k = (
        math.ceil(
            (W + 2 * attitude_tolerance - fov)
            / (fov * (1 - overlap) - attitude_tolerance)
        )
        + 1
    )

    if 1:
        # optional sanity checks

        stride = (W + 2 * attitude_tolerance - fov) / (k - 1)
        assert_lte(
            stride, fov * (1 - overlap) - attitude_tolerance, EPS
        )  # sufficient overlap

        # check if we have more images than necessary
        if k == 1:
            pass  # obviously need at least one image
        elif k == 2:
            assert_lte(fov, W + 2 * attitude_tolerance, EPS)  # k = 1 is not enough
        else:
            stride1 = (W + 2 * attitude_tolerance - fov) / (k - 2)
            assert_gte(
                stride1, fov * (1 - overlap) - attitude_tolerance, EPS
            )  # k is minimized

    min_center = -(range_radius + attitude_tolerance) + fov / 2
    max_center = -min_center
    return np.linspace(min_center, max_center, num=k)


def pano_1d_complete_pan(fov, overlap, attitude_tolerance):
    """
    Returns image center coordinates such that the images cover the full pan
    range -180 .. 180 with at least the specified @overlap between all consecutive images,
    including at the wrap-around, and the image sequence is centered at pan = 0.

    :param float fov: Field of view of each image (degrees).
    :param float overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
    :param float attitude_tolerance: Ensure overlap criterion is met even if relative attitude between a pair of adjacent images, or between an image and the desired pano boundary, is off by at most this much (degrees).
    :return: A vector of orientations of image centers (degrees).
    """
    k = math.ceil(360 / (fov * (1 - overlap) - attitude_tolerance))
    centers = np.linspace(-180, 180, num=k, endpoint=False)
    # ensure pano is centered at pan = 0
    mid_point = np.mean((centers[0], centers[-1]))
    return centers - mid_point


def pano_1d_pan(pan_radius, h_fov, v_fov, overlap, attitude_tolerance, tilt):
    """
    Returns image center coordinates for a row of images at tilt value
    @tilt that cover the range -@pan_radius .. +@pan_radius with at least
    the specified @overlap. Special complete wrap-around behavior is
    triggered when @pan_radius is exactly 180.

    :param float pan_radius: Cover pan angle of -pan_radius to +pan_radius (degrees).
    :param float h_fov: Horizontal field of view of each image (degrees).
    :param float v_fov: Vertical field of view of each image (degrees).
    :param float overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
    :param float attitude_tolerance: Ensure overlap criterion is met even if relative attitude between a pair of adjacent images is off by at most this much (degrees).
    :param float tilt: The tilt of the image center (degrees).
    :return: A vector of pan orientations of image centers (degrees).
    """
    h_fov_effective = get_h_fov_effective(h_fov, v_fov, tilt)
    # print("tilt=%s h_fov_effective=%s" % (tilt, h_fov_effective))
    if pan_radius == 180:
        return pano_1d_complete_pan(h_fov_effective, overlap, attitude_tolerance)
    else:
        return pano_1d(pan_radius, h_fov_effective, overlap, attitude_tolerance)


def pano_orientations(
    pan_radius, tilt_radius, h_fov, v_fov, overlap, attitude_tolerance
):
    """
    Return image center coordinates that cover the specified pan and tilt ranges,
    with the specified @overlap. Special complete wrap-around behavior is triggered when
    the pan_radius is exactly 180.

    :param float pan_radius: Cover pan angle of -pan_radius to +pan_radius (degrees).
    :param float tilt_radius: Cover tilt angle of -tilt_radius to +tilt_radius (degrees).
    :param float h_fov: Horizontal field of view of each image (degrees).
    :param float v_fov: Vertical field of view of each image (degrees).
    :param float overlap: Minimum required overlap between consecutive images, as a proportion of the image field of view (0 .. 1).
    :param float attitude_tolerance: Ensure overlap criterion is met even if relative attitude between a pair of adjacent images is off by at most this much (degrees).
    :return: (image_centers, nrows, ncols). A list of orientations of image centers, the number of rows in the panorama, and the number of columns.
    """

    # calculate all image centers
    image_centers = []
    tilt_vals = pano_1d(tilt_radius, v_fov, overlap, attitude_tolerance)
    for iy, tilt in enumerate(reversed(tilt_vals)):
        pan_vals = pano_1d_pan(
            pan_radius, h_fov, v_fov, overlap, attitude_tolerance, tilt
        )
        for pan in pan_vals:
            image_centers.append((pan, tilt, iy, -1))
    # image_centers = np.array(image_centers)
    # print(image_centers.shape)
    image_centers = np.array(
        image_centers,
        dtype=[
            ("pan", np.double),
            ("tilt", np.double),
            ("iy", np.int16),
            ("ix", np.int16),
        ],
    )

    # assign image centers to columns
    min_tilt = np.min(np.abs(tilt_vals))
    column_centers = pano_1d_pan(
        pan_radius, h_fov, v_fov, overlap, attitude_tolerance, min_tilt
    )
    image_pans = image_centers["pan"]
    image_centers["ix"] = np.array(
        [np.argmin(np.abs(column_centers - p)) for p in image_pans]
    )

    # order images in column-major order; alternate direction top-to-bottom or bottom-to-top
    images_ordered = []
    for ix, pan in enumerate(column_centers):
        images_in_col_ind = image_centers["ix"] == ix
        images_in_col = image_centers[images_in_col_ind]

        # sort on tilt bottom-to-top
        images_in_col = images_in_col[np.argsort(images_in_col["tilt"])]

        # on even-numbered columns, reverse tilt order, top-to-bottom
        if ix % 2 == 0:
            images_in_col = images_in_col[::-1]

        images_ordered += list(images_in_col)
    images_ordered = np.array(images_ordered)

    nrows = len(tilt_vals)
    ncols = len(column_centers)

    return images_ordered, nrows, ncols


def print_pano(pano):
    image_centers, nrows, ncols = pano
    num_images = image_centers.shape[0]
    print(
        "%s images, %s rows x %s cols, frame# [pan tilt]:" % (num_images, nrows, ncols)
    )
    if 0:
        print(image_centers)
        return
    image_lookup = {
        (iy, ix): (i, pan, tilt) for i, (pan, tilt, iy, ix) in enumerate(image_centers)
    }
    for iy in range(nrows):
        print("  ", end="")
        for ix in range(ncols):
            image_info = image_lookup.get((iy, ix))
            if image_info is None:
                print("              ", end="   ")
            else:
                i, pan, tilt = image_info
                print("%2d [%4d %4d]" % (i, round(pan), round(tilt)), end="   ")
        print()


def assert_equal(a, b, eps):
    assert abs(a - b) < eps, "FAIL: %s should equal %s, within tolerance %s" % (
        a,
        b,
        eps,
    )


def assert_lte(a, b, eps):
    assert a <= b + eps, "FAIL: %s should be <= %s, within tolerance %s" % (a, b, eps)


def assert_gte(a, b, eps):
    assert a >= b - eps, "FAIL: %s should be >= %s, within tolerance %s" % (a, b, eps)


def test_case(config):
    label = config["label"]
    print(label, end=": ")
    pano = pano_orientations(
        float(config["pan_radius_degrees"]),
        float(config["tilt_radius_degrees"]),
        float(config["h_fov_degrees"]),
        float(config["v_fov_degrees"]),
        float(config["overlap"]),
        float(config["attitude_tolerance_degrees"]),
    )
    print_pano(pano)
    print()


def do_cases(csv_path):
    with open(csv_path, "r") as csv_stream:
        rows = csv.DictReader(csv_stream)
        for row in rows:
            test_case(row)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "in_csv",
        nargs="?",
        help="input file of test cases",
        default="pano_test_cases.csv",
    )

    args = parser.parse_args()
    level = logging.INFO if args.verbose else logging.WARN
    logging.basicConfig(level=level, format="%(message)s")
    do_cases(args.in_csv)
