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

# Create camera files that texrecon will understand.

import argparse
import glob
import os
import re
import sys

import numpy as np

parser = argparse.ArgumentParser(
    description="Convert cameras to the format of texrecon."
)
parser.add_argument(
    "--camera_dir",
    default="",
    help="The directory containing the camera information (the output of geometry_mapper).",
)
parser.add_argument(
    "--undistorted_image_dir",
    default="",
    help="The directory containing the undistorted images.",
)
parser.add_argument(
    "--camera_type",
    default="",
    help="The camera type (nav_cam, haz_cam, or sci_cam, etc.).",
)

args = parser.parse_args()

if args.camera_dir == "" or args.undistorted_image_dir == "" or args.camera_type == "":
    print(
        "Must specify the camera directory, directory of undistorted images, and camera type."
    )
    sys.exit(1)

# Read the intrinsics
intr_file = args.undistorted_image_dir + "/undistorted_intrinsics.txt"
if not os.path.exists(intr_file):
    print("Missing file: " + intr_file)
    sys.exit(1)
with open(intr_file, "r") as f:
    for line in f:
        if re.match("^\s*\#", line):
            continue  # ignore the comments
        vals = line.split()
        if len(vals) < 5:
            print("Expecting 5 parameters in " + intr_file)
            sys.exit(1)
        widx = float(vals[0])
        widy = float(vals[1])
        f = float(vals[2])
        cx = float(vals[3])
        cy = float(vals[4])

        max_wid = widx
        if widy > max_wid:
            max_wid = widy

        # normalize
        nf = f / max_wid
        ncx = cx / widx
        ncy = cy / widy
        d0 = 0.0
        d1 = 0.0
        paspect = 1.0
        break  # finished reading the line we care for

# Convert the cameras to texrecon's format
suffix = "_" + args.camera_type + "_to_world.txt"

# Get the cameras to write based on the list of images in the index
# We avoid simply doing an ls in that directory to ensure we don't
# run into old files
index_file = os.path.join(args.camera_dir, args.camera_type + "_index.txt")

camera_files = []

with open(index_file, "r") as f:
    for image_file in f:
        image_file = image_file.rstrip()
        image_file = os.path.basename(image_file)

        m = re.match("^(.*?)\.jpg", image_file)
        if not m:
            print("Expecting a .jpg file, but got: " + image_file)

        in_cam = os.path.join(args.camera_dir, m.group(1) + suffix)
        camera_files.append(in_cam)

        out_cam = args.undistorted_image_dir + "/" + os.path.basename(in_cam)

        m = re.match("^(.*?)" + suffix, out_cam)
        if not m:
            print("Could not match desired expression.")
            sys.exit(1)
        out_cam = m.group(1) + ".cam"

        if not os.path.exists(in_cam):
            print("Cannot find: " + in_cam)
            sys.exit(1)

        M = np.loadtxt(in_cam)  # camera to world
        M = np.linalg.inv(M)  # world to camera

        print("Writing: " + out_cam)
        with open(out_cam, "w") as g:

            # translation
            g.write("%0.17g %0.17g %0.17g " % (M[0][3], M[1][3], M[2][3]))

            # rotation
            g.write(
                "%0.17g %0.17g %0.17g %0.17g %0.17g %0.17g %0.17g %0.17g %0.17g\n"
                % (
                    M[0][0],
                    M[0][1],
                    M[0][2],
                    M[1][0],
                    M[1][1],
                    M[1][2],
                    M[2][0],
                    M[2][1],
                    M[2][2],
                )
            )

            # normaized inrinsics
            g.write(
                "%0.17g %0.17g %0.17g %0.17g %0.17g %0.17g\n"
                % (nf, d0, d1, paspect, ncx, ncy)
            )

# Save the name of the camera transforms. This will be used later
# for individual texturing of each image and camera.
camera_list = os.path.join(args.camera_dir, args.camera_type + "_transforms.txt")
with open(camera_list, "w") as f:
    print("Writing: " + camera_list)
    for camera in camera_files:
        f.write(camera + "\n")
