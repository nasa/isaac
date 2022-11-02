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
Convenience script for converting image formats (with optional resizing) using
OpenCV. You may find it useful for large panorama images because OpenCV has
larger image size limits than ImageMagick convert.

Example:
  rosrun inspection scripts/opencv_convert.py pano.png -resize 4000x4000 pano_small.jpg
"""

import argparse

import cv2


def opencv_convert(in_path, out_path, resize):
    im = cv2.imread(in_path, cv2.IMREAD_UNCHANGED)

    if resize:
        # parse input string
        w_str, h_str = resize.split("x")
        w = int(w_str)
        h = int(h_str)

        # preserve aspect ratio, interpreting requested dimensions as limits on
        # output width and height, like ImageMagick convert.
        in_h, in_w = im.shape[:2]
        scale_factor = min(float(w) / in_w, float(h) / in_h)
        resize_aspect = tuple([int(round(scale_factor * val)) for val in (in_w, in_h)])

        # perform resize operation
        im = cv2.resize(im, resize_aspect, interpolation=cv2.INTER_CUBIC)

    cv2.imwrite(out_path, im)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "in_image",
        type=str,
        help="input image path",
    )
    parser.add_argument(
        "out_image",
        type=str,
        help="output image path",
    )
    parser.add_argument(
        "-r",
        "--resize",
        type=str,
        default=None,
        help="limit output size to specified dimensions, like '640x480'. Input aspect ratio will be preserved within the specified limits.",
    )
    args = parser.parse_args()

    opencv_convert(args.in_image, args.out_image, args.resize)


if __name__ == "__main__":
    main()
