#!/usr/bin/env python
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
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
Compare PTO files.
"""

import argparse
import array
import os

import hsi
import matplotlib
import numpy as np

matplotlib.use("Agg")  # must call before importing pyplot
from matplotlib import pyplot as plt


def read_pto(pano, pto_path):
    ifs = hsi.ifstream(pto_path)
    pano.readData(ifs)


def get_params(pto_path, x_param, y_param, lonlat):
    pano = hsi.Panorama()
    read_pto(pano, pto_path)

    result_in = array.array("d")
    for i in range(pano.getNrOfImages()):
        img = pano.getImage(i)
        result_in.extend((img.getVar(x_param), img.getVar(y_param)))

    result = np.array(result_in, dtype=np.float64).reshape((-1, 2))
    if lonlat:
        result = normalize_lonlat(result)
    return result


def xyz_from_lonlat(lonlat):
    lon = lonlat[:, 0] * np.pi / 180
    lat = lonlat[:, 1] * np.pi / 180
    result = np.array(
        (np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat))
    ).T
    return result


def lonlat_from_xyz(xyz):
    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]
    lon = np.arctan2(y, x)
    lat = np.arctan2(z, np.sqrt(x * x + y * y))
    return np.array((lon * 180 / np.pi, lat * 180 / np.pi)).T


def normalize_lonlat(lonlat):
    return lonlat_from_xyz(xyz_from_lonlat(lonlat))


def lon_wrap_line(x_line, y_line):
    x1, x2 = x_line
    y1, y2 = y_line
    if abs(x1 - x2) > 180:
        if x1 < x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
        x2p = x2 + 360
        yb = y1 + (y2 - y1) * (180 - x1) / (x2p - x1)
        return (x1, 180, -180, x2), (y1, yb, yb, y2)
    else:
        return x_line, y_line


def lon_wrap_lines(x, y):
    xa = array.array("d")
    ya = array.array("d")
    for x_line, y_line in zip(x.T, y.T):
        x_lines, y_lines = lon_wrap_line(x_line, y_line)
        xa.extend(x_lines)
        ya.extend(y_lines)
    x_out = np.array(xa, dtype=np.float64).reshape((-1, 2)).T
    y_out = np.array(ya, dtype=np.float64).reshape((-1, 2)).T
    return x_out, y_out


def plot_params(ptos, x_param, y_param, out_path, lonlat=False):
    fig, ax = plt.subplots()
    prev_data = None

    for pto in ptos:
        data = get_params(pto, x_param, y_param, lonlat)
        plt.plot(data[:, 0], data[:, 1], "+")
        if prev_data is not None:
            x = np.array([prev_data[:, 0], data[:, 0]])
            y = np.array([prev_data[:, 1], data[:, 1]])
            if lonlat:
                x, y = lon_wrap_lines(x, y)
            plt.plot(x, y, "-", color="gray")
        prev_data = data

    pos = ax.get_position()
    ax.set_position([pos.x0, pos.y0, pos.width * 0.7, pos.height])
    pto_names = [os.path.splitext(os.path.basename(pto))[0] for pto in ptos]
    ax.legend(pto_names, loc="upper left", bbox_to_anchor=(1.05, 1))
    plt.xlabel(x_param)
    plt.ylabel(y_param)

    if lonlat:
        tick_size = 30
        ax.set_xticks(range(-180, 180 + tick_size, tick_size))
        ax.set_yticks(range(-90, 90 + tick_size, tick_size))
        ax.axis([-180, 180, -90, 90])

    plt.grid()
    ax.axis("scaled")

    # plt.tight_layout()
    fig.set_size_inches((10, 5))

    plt.savefig(out_path)
    print("Wrote plot to %s" % out_path)
    plt.close()


def compare_pto(pto_paths):
    plot_params(pto_paths, "y", "p", "compare_pto_y_p.png", lonlat=True)
    plot_params(pto_paths, "Tpy", "Tpp", "compare_pto_Tpy_Tpp.png", lonlat=True)
    plot_params(pto_paths, "y", "r", "compare_pto_y_r.png")


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    pass


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "pto",
        nargs="+",
        help="input pto path",
    )
    args = parser.parse_args()

    compare_pto(args.pto)


if __name__ == "__main__":
    main()
