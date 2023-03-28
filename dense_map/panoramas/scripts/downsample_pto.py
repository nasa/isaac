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
Downsample pano input frames, metadata, control points, and requested output
size in PTO. Convenient to speed up the rest of the stitching process when
debugging.

Example:
  rosrun inspection scripts/downsample_pto.py in.pto --factor 4 out.pto
"""

import argparse
import os

import cv2
import hsi


def downsample_frame(in_path, out_path, factor):
    print("  downsample_frame %s %s" % (in_path, out_path))

    if os.path.exists(out_path):
        print("    output file already exists, skipping")
        return

    im = cv2.imread(in_path, cv2.IMREAD_UNCHANGED)
    rows, cols = im.shape[:2]
    down_size = (int(cols / factor), int(rows / factor))
    down_im = cv2.resize(im, down_size, interpolation=cv2.INTER_LINEAR)
    cv2.imwrite(out_path, down_im)


def read_pto(pano, pto_path):
    ifs = hsi.ifstream(pto_path)
    pano.readData(ifs)


def write_pto(pano, pto_path):
    ofs = hsi.ofstream(pto_path)
    pano.writeData(ofs)


def downsampled_path(in_path):
    in_prefix, in_ext = os.path.splitext(in_path)
    return in_prefix + "_down" + in_ext


def downsample_pto(in_path, out_path, factor):
    if os.path.exists(out_path):
        print("Output file %s exists, not overwriting." % out_path)
        return

    pano = hsi.Panorama()
    read_pto(pano, in_path)

    # process input images
    print("Downsampling:")
    for i in range(pano.getNrOfImages()):
        img = pano.getImage(i)
        orig_path = img.getFilename()
        down_path = downsampled_path(orig_path)

        # downsample image
        downsample_frame(orig_path, down_path, factor)

        # update pto: point to dowsampled image
        img.setFilename(down_path)

        # update pto: image width and height
        img_size = img.getSize()
        img_size.x = int(img_size.x / factor)
        img_size.y = int(img_size.y / factor)
        img.setSize(img_size)

    # process control points
    pts = pano.getCtrlPoints()
    for pt in pts:
        pt.x1 /= factor
        pt.x2 /= factor
        pt.y1 /= factor
        pt.y2 /= factor
    pano.setCtrlPoints(pts)

    # reduce size of output pano
    opts = pano.getOptions()
    w, h = opts.getWidth(), opts.getHeight()
    # Note: the setWidth() call sometimes auto-updates height, so it's important
    # that we queried both w and h before setting either parameter.
    opts.setWidth(int(w / factor))
    opts.setHeight(int(h / factor))

    write_pto(pano, out_path)
    print("Wrote to %s" % out_path)


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    pass


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "in_pto",
        type=str,
        help="input pto path",
    )
    parser.add_argument(
        "out_pto",
        type=str,
        help="output pto path",
    )
    parser.add_argument(
        "-f",
        "--factor",
        type=float,
        default=8,
        help="downsample by specified factor",
    )
    args = parser.parse_args()

    downsample_pto(args.in_pto, args.out_pto, args.factor)


if __name__ == "__main__":
    main()
