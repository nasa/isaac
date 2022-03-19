#!/usr/bin/env python
#
# Copyright (c) 2021, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
# platform" software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

"""
Debugs pano coverage.

Reads pano_test_cases.csv to extract test case parameters and the
case_*.csv panorama coverage plans output by the test_pano tool.

For each test case:

- Checks if the panorama satisfies the coverage requirement and the
  image overlap requirement (tolerating pointing error up to attitude
  tolerance).
- Outputs several plots to help understand coverage.

Note that, unfortunately, the pano_orientations() function does *not* guarantee
its output satisfies the coverage and overlap requirements. This is because
spherical trig warps the image area to be a non-rectangle in a way that's
difficult to correct for in general. You can use this tool to check if there is
a problem with an output plan. If there is, you can encourage the coverage
planner to add more images by adding some extra "warp margin" padding to
plan_attitude_tolerance_degrees while leaving test_attitude_tolerance_degrees
unchanged, until the problem is resolved.
"""

from __future__ import print_function

import argparse
import itertools
import logging

import matplotlib
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

if __name__ == "__main__":
    matplotlib.use("Agg")  # must come before importing pyplot

from matplotlib import collections as mc
from matplotlib import colors as mcolors
from matplotlib import pyplot as plt

EPS = 1e-5
TEST_GRID_SPACING = 5
FRAME_TEST_GRID_COUNT = 20


def from_pan_tilt(pan, tilt):
    return Rotation.from_euler("XYZ", (0, tilt, pan), degrees=True)


def to_pan_tilt(rot):
    rpy = rot.as_euler("XYZ", degrees=True)
    return rpy[2], rpy[1]


def add_pan_tilt1(pt, dpt):
    rot = from_pan_tilt(*pt)
    drot = from_pan_tilt(*dpt)
    return np.array(to_pan_tilt(drot * rot))


add_pan_tilt = np.vectorize(add_pan_tilt1, signature="(2),(2)->(2)")


def annotate_test_cases(test_cases):
    num_images = []
    nrows = []
    ncols = []
    for label in test_cases["label"]:
        pano = pd.read_csv("case_%s.csv" % label)
        num_images.append(len(pano))
        nrows.append(pano["iy"].max() + 1)
        ncols.append(pano["ix"].max() + 1)
    test_cases["num_images"] = num_images
    test_cases["nrows"] = nrows
    test_cases["ncols"] = ncols


def get_test_case(in_csv_path, label):
    test_cases = pd.read_csv(in_csv_path)
    params = next(test_cases[test_cases["label"] == label].itertuples())
    pano = pd.read_csv("case_%s.csv" % label)
    if params.pan_radius_degrees == 180:
        pan_radius_adjusted = 180
    else:
        pan_radius_adjusted = (
            params.pan_radius_degrees + 0.5 * params.test_attitude_tolerance_degrees
        )
    return {
        "label": label,
        "params": params,
        "pano": pano,
        "h_fov_adjusted": params.h_fov_degrees - params.test_attitude_tolerance_degrees,
        "v_fov_adjusted": params.v_fov_degrees - params.test_attitude_tolerance_degrees,
        "pan_radius_adjusted": pan_radius_adjusted,
        "tilt_radius_adjusted": params.tilt_radius_degrees
        + 0.5 * params.test_attitude_tolerance_degrees,
    }


def pairwise(seq):
    """
    Like itertools.pairwise(), which is not available in some Python versions.
    """
    first = True
    for val in seq:
        if first:
            first = False
        else:
            yield prev, val
        prev = val


def plot_box(ws, en, style="k-"):
    w, s = ws
    e, n = en
    corners = [
        (w, s),
        (w, n),
        (e, n),
        (e, s),
    ]
    for p0, p1 in pairwise(corners + [corners[0]]):
        x0, y0 = p0
        x1, y1 = p1
        plt.plot([x0, x1], [y0, y1], style)


def plot_style_lon_lat(fig, ax):
    """
    If your plot (x, y) can be interpreted as (lon, lat) covering the whole sphere,
    use this function to set up nice plot bounds and grid ticks.
    """
    ax.axis("equal")
    plt.xticks(np.arange(-180, 181, 10))
    plt.yticks(np.arange(-90, 91, 10))
    ax.set_xlim((-186, 186))
    ax.set_ylim((-93, 93))
    plot_box((-180, -90), (180, 90))
    plt.grid()
    plt.xlabel("Pan (degrees)")
    plt.ylabel("Tilt (degrees)")
    fig.set_size_inches((20, 10))


def plot_required_coverage(test_case):
    pan_radius = test_case["pan_radius_adjusted"]
    tilt_radius = test_case["tilt_radius_adjusted"]
    plot_box((-pan_radius, -tilt_radius), (pan_radius, tilt_radius), style="g--")


def plot_pano_seq(test_case, out_path=None):
    """
    Plot sequence of frame centers.
    """
    fig, ax = plt.subplots()
    pano = test_case["pano"]
    plt.plot(pano["pan"], pano["tilt"], "o-")
    plt.plot(pano["pan"][0], pano["tilt"][0], "ro")  # first frame red
    for frame in pano.itertuples():
        ax.annotate(frame.Index, (frame.pan + 3, frame.tilt + 3))
    plot_required_coverage(test_case)
    plot_style_lon_lat(fig, ax)
    plt.title("Sequence of image center pan/tilt values")
    if out_path:
        plt.savefig(out_path)
        logging.info("plot_pano_seq: wrote to %s", out_path)
        plt.close()


def linspace_pts(pt0, pt1, k=20):
    x = np.linspace(0, 1, k)[:, np.newaxis]
    return (1 - x) * pt0[np.newaxis, :] + x * pt1[np.newaxis, :]


def get_image_borders_dpt_internal(h_fov, v_fov):
    h_rad = 0.5 * h_fov
    v_rad = 0.5 * v_fov
    corners = [
        np.array([-h_rad, -v_rad]),
        np.array([-h_rad, v_rad]),
        np.array([h_rad, v_rad]),
        np.array([h_rad, -v_rad]),
    ]
    result = np.zeros((0, 2))
    for pt0, pt1 in pairwise(corners + [corners[0]]):
        result = np.vstack((result, linspace_pts(pt0, pt1)))
    return result


def get_image_borders_dpt(test_case):
    return get_image_borders_dpt_internal(
        test_case["h_fov_adjusted"], test_case["v_fov_adjusted"]
    )


def plot_frame_borders(ax, fborders, color):
    fborder_pairs = np.column_stack([fborders[:-1, :], fborders[1:, :]])
    long_segments = np.abs(fborder_pairs[:, 0] - fborder_pairs[:, 2]) > 270
    fborder_pairs = fborder_pairs[~long_segments]
    fbp_list = [((x0, y0), (x1, y1)) for x0, y0, x1, y1 in fborder_pairs]
    mcolor = mcolors.to_rgba(color)
    lc = mc.LineCollection(fbp_list, colors=[mcolor] * len(fbp_list))
    ax.add_collection(lc)


def plot_pano_borders(test_case, out_path=None):
    fig, ax = plt.subplots()
    borders = get_image_borders_dpt(test_case)
    colors = itertools.cycle(
        ("r", "g", "b", "m", "k", "c", "#444400", "#440044", "#004444")
    )
    pano = test_case["pano"]
    for frame in pano.itertuples():
        pt = np.array([frame.pan, frame.tilt])
        fborders = add_pan_tilt(pt, borders)
        plot_frame_borders(ax, fborders, next(colors))
    plot_style_lon_lat(fig, ax)
    plot_required_coverage(test_case)
    plt.title("Pano image borders (FOV reduced to represent attitude tolerance)")
    if out_path:
        plt.savefig(out_path)
        logging.info("plot_pano_borders: wrote to %s", out_path)
        plt.close()


def get_bounds_checker(image_pt, h_fov, v_fov):
    h_radius = 0.5 * h_fov
    v_radius = 0.5 * v_fov
    image_rot = from_pan_tilt(*image_pt)

    def in_bounds_pt(pt):
        pt_rot = from_pan_tilt(*pt)
        drot = pt_rot * image_rot.inv()
        dpan, dtilt = to_pan_tilt(drot)
        result_bool = abs(dtilt) < v_radius + EPS
        if result_bool:
            if abs(90 - abs(dtilt)) < 0.1:
                pass  # pan check meaningless at pole
            else:
                result_bool &= abs(dpan) < h_radius + EPS
        return 1 if result_bool else 0

    return np.vectorize(in_bounds_pt, signature="(2)->()")


def get_global_test_grid(test_case):
    pan_radius = test_case["pan_radius_adjusted"]
    tilt_radius = test_case["tilt_radius_adjusted"]

    k_pan = int(pan_radius * 2 / TEST_GRID_SPACING) + 1
    pan = np.linspace(-pan_radius, pan_radius, k_pan)

    # check northern hemisphere only. southern is symmetrical.
    k_tilt = int(tilt_radius / TEST_GRID_SPACING) + 1
    tilt = np.linspace(0, tilt_radius, k_tilt)

    tt, pp = np.meshgrid(tilt, pan)
    return np.dstack((pp, tt))


def get_coverage_count(test_case, test_grid):
    coverage_count = np.zeros(test_grid.shape[:-1])
    pano = test_case["pano"]
    for frame in pano.itertuples():
        fpt = (frame.pan, frame.tilt)
        in_frame_bounds = get_bounds_checker(
            fpt, test_case["h_fov_adjusted"], test_case["v_fov_adjusted"]
        )
        coverage_count = coverage_count + in_frame_bounds(test_grid)
    return coverage_count


def plot_hist_integer(arr, out_path=None):
    plt.hist(arr.flatten(), bins=np.arange(arr.min() - 1, arr.max() + 1) + 0.5)
    plt.xticks(np.arange(-1, arr.max() + 1))
    plt.title("Coverage count frequency histogram")
    plt.xlabel("Number of pano images that include test point")
    plt.ylabel("Number of test points")
    if out_path:
        plt.savefig(out_path)
        logging.info("plot_hist_integer: wrote to %s", out_path)
        plt.close()


def plot_coverage_count(test_case, test_grid, coverage_count, cblabel, out_path=None):
    fig, ax = plt.subplots()
    plt.scatter(test_grid[:, :, 0], test_grid[:, :, 1], c=coverage_count)
    plot_style_lon_lat(fig, ax)
    plot_required_coverage(test_case)
    plt.colorbar(label=cblabel)
    plt.title("Coverage count map")
    if out_path:
        plt.savefig(out_path)
        logging.info("plot_coverage_count: wrote to %s", out_path)
        plt.close()


def check_complete_coverage(coverage_count):
    num_uncovered = np.count_nonzero(coverage_count == 0)
    success = num_uncovered == 0
    print("check_complete_coverage:", "PASS" if success else "FAIL")
    return success


def pan_distance(d):
    abs_d = np.abs(d)
    return np.minimum(abs_d, 360 - abs_d)


def get_manhattan_neighbors_internal(frame, params, frameLookup, nrows, ncols):
    iy = frame.iy
    ix = frame.ix
    neighbors = []

    north_south = (
        ("north", (0 <= (iy - 1)), -1),
        ("south", ((iy + 1) < nrows), 1),
    )
    for dir_label, bound_check, dy in north_south:
        if not bound_check:
            continue
        neighbor_y = iy + dy
        neighbor_row = [f for f in frameLookup.values() if f.iy == neighbor_y]
        neighbor_row_pan = np.array([f.pan for f in neighbor_row])
        sort_ind = np.argsort(pan_distance(neighbor_row_pan - frame.pan))
        neighbor1 = neighbor_row[sort_ind[0]]
        dir_neighbors = [neighbor1]
        if pan_distance(neighbor1.pan - frame.pan) > 0.1 and len(neighbor_row) > 1:
            # columns not aligned, need the other bracketing neigbor
            dir_neighbors.append(neighbor_row[sort_ind[1]])
        neighbors.append((dir_label, dir_neighbors))

    west_east = (
        ("west", -1),
        ("east", 1),
    )
    for dir_label, dx in west_east:
        x_vals = ix + dx * np.arange(ncols - 1)
        if params.pan_radius_degrees == 180:
            # implement 360 wrap-around
            x_vals = np.mod(x_vals, ncols)
        else:
            # stop at west/east edges
            x_vals = x_vals[(0 <= x_vals) & (x_vals < ncols)]
        for x in x_vals:
            if x == ix:
                continue
            neighbor = frameLookup.get((iy, x))
            if neighbor is not None:
                neighbors.append((dir_label, [neighbor]))
                break
    return neighbors


def get_manhattan_neighbors(frame, test_case):
    params = test_case["params"]
    pano = test_case["pano"]
    frameLookup = {(frame.iy, frame.ix): frame for frame in pano.itertuples()}
    # print(frameLookup)
    nrows = pano["iy"].max() + 1
    ncols = pano["ix"].max() + 1
    return get_manhattan_neighbors_internal(frame, params, frameLookup, nrows, ncols)


def get_frame_test_grid(pt, h_fov, v_fov, k=FRAME_TEST_GRID_COUNT):
    pan = np.linspace(-0.5 * h_fov, 0.5 * h_fov, k)
    tilt = np.linspace(-0.5 * v_fov, 0.5 * v_fov, k)
    tt, pp = np.meshgrid(tilt, pan)
    test_grid_dpt = np.dstack((pp, tt))
    return add_pan_tilt(pt, test_grid_dpt)


def get_in_frame(test_grid, frame, h_fov, v_fov):
    bounds_checker = get_bounds_checker((frame.pan, frame.tilt), h_fov, v_fov)
    return bounds_checker(test_grid)


def frame_text(frame):
    return "%s [%s %s]" % (frame.Index, int(round(frame.pan)), int(round(frame.tilt)))


def check_overlap(test_case, verbose=False):
    params = test_case["params"]
    h_fov = test_case["h_fov_adjusted"]
    v_fov = test_case["v_fov_adjusted"]
    required_overlap = params.overlap
    pano = test_case["pano"]
    min_overlap = 999
    for frame in pano.itertuples():
        # analyze northern hemisphere only to save time. southern is symmetrical.
        if frame.tilt < 0:
            continue
        frame_grid = get_frame_test_grid((frame.pan, frame.tilt), h_fov, v_fov)
        neighbors = get_manhattan_neighbors(frame, test_case)
        if verbose:
            print("Frame %s" % (frame,))
        for dir_label, dir_neighbors in neighbors:
            in_dir_neighbors_count = np.zeros(
                (frame_grid.shape[0], frame_grid.shape[1])
            )
            for neighbor in dir_neighbors:
                in_dir_neighbors_count += get_in_frame(
                    frame_grid, neighbor, h_fov, v_fov
                )
            num_overlap = np.count_nonzero(in_dir_neighbors_count)
            num_total = in_dir_neighbors_count.size
            overlap = float(num_overlap) / num_total
            if overlap < min_overlap:
                min_overlap = overlap
                min_info = frame, dir_label, dir_neighbors
            if verbose:
                success = overlap > required_overlap
                success_text = "PASS" if success else "FAIL"
                print(
                    "  %s overlap=%d%% %s %s"
                    % (
                        success_text,
                        round(overlap * 100),
                        dir_label,
                        ", ".join(map(frame_text, dir_neighbors)),
                    )
                )

    print("\nMinimum overlap %s%%, at:" % int(min_overlap * 100))

    frame, dir_label, dir_neighbors = min_info
    print("Frame: ", frame)
    print("Neighbors in direction: ", dir_label)
    print("  %s" % dir_neighbors)
    print()

    global_success = min_overlap > required_overlap
    global_success_text = "PASS" if global_success else "FAIL"
    print("check_overlap:", global_success_text)
    return global_success


# debug check_overlap()
def plot_overlap_example(test_case):
    pano = test_case["pano"]
    h_fov, v_fov = test_case["h_fov_adjusted"], test_case["v_fov_adjusted"]
    seq = pano.itertuples()
    # arbitrarily select first two frames in the pano
    frame = next(seq)
    neighbor = next(seq)
    frame_grid = get_frame_test_grid((frame.pan, frame.tilt), h_fov, v_fov)
    bounds_checker = get_bounds_checker((neighbor.pan, neighbor.tilt), h_fov, v_fov)
    in_neighbor = bounds_checker(frame_grid)
    plot_coverage_count(
        test_case, frame_grid, in_neighbor, "Neighbor image includes point"
    )
    plt.title("Example analysis of overlap between neighboring images")
    num_overlap = np.count_nonzero(in_neighbor)
    num_total = in_neighbor.size
    neighbor_overlap = float(num_overlap) / num_total
    print("%s / %s (%s%%)" % (num_overlap, num_total, round(neighbor_overlap * 100)))


def check_test_case(test_case, verbose=False):
    label = test_case["label"]
    print("=== check_test_case %s ===" % label)

    plot_pano_seq(test_case, "plot_%s_seq.png" % label)
    plot_pano_borders(test_case, "plot_%s_borders.png" % label)

    test_grid = get_global_test_grid(test_case)
    coverage_count = get_coverage_count(test_case, test_grid)
    plot_hist_integer(coverage_count, "plot_%s_coverage_hist.png" % label)
    plot_coverage_count(
        test_case,
        test_grid,
        coverage_count,
        "Number of pano images that include test point",
        "plot_%s_coverage_count.png" % label,
    )

    success = True
    success &= check_complete_coverage(coverage_count)
    success &= check_overlap(test_case, verbose)
    success_text = "PASS" if success else "FAIL"
    print()
    print("check_test_case %s: %s" % (label, success_text))
    print()
    return success


def do_plot_pano(in_csv_path, verbose, case_label):
    test_cases = pd.read_csv(in_csv_path)
    success = True
    if case_label:
        test_case = get_test_case(in_csv_path, case_label)
        success &= check_test_case(test_case, verbose)
    else:
        for label in test_cases["label"]:
            test_case = get_test_case(in_csv_path, label)
            success &= check_test_case(test_case, verbose)
    success_text = "PASS" if success else "FAIL"
    print()
    print("overall do_plot_pano:", success_text)
    print()


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
    parser.add_argument(
        "-v",
        "--verbose",
        help="print more debug info",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "-c",
        "--case",
        nargs="?",
        help="Process only the specified case, instead of all cases",
    )

    args = parser.parse_args()
    level = logging.INFO if args.verbose else logging.WARN
    logging.basicConfig(level=level, format="%(message)s")
    do_plot_pano(args.in_csv, args.verbose, args.case)
