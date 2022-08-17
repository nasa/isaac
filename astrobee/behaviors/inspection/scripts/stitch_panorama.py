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

import argparse
import os
import re
import subprocess
import sys

import rosbag
from hsi import *  # load hugin
from tf.transformations import euler_from_quaternion, quaternion_from_euler

RAD2DEG = 180 / 3.1415


def run_cmd(cmd, logfile=""):

    print((" ".join(cmd)))

    if logfile != "":
        f = open(logfile, "w")

    stdout = ""
    stderr = ""
    popen = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True
    )
    for stdout_line in iter(popen.stdout.readline, ""):
        if logfile != "":
            f.write(stdout_line)

        stdout += stdout_line

    popen.stdout.close()
    return_code = popen.wait()
    if return_code:
        raise subprocess.CalledProcessError(return_code, cmd)

    if return_code != 0:
        print(("Failed to run command.\nOutput: ", stdout, "\nError: ", stderr))

    return (return_code, stdout, stderr)


def parse_args():

    parser = argparse.ArgumentParser(description="Generates/updates hugin files.")
    parser.add_argument(
        "-bag_name",
        type=str,
        required=True,
        help="Input bagfile..",
    )
    parser.add_argument(
        "-input_hugin",
        type=str,
        required=False,
        help="Input Hugin pto file.",
    )
    parser.add_argument(
        "-output_hugin",
        type=str,
        required=False,
        help="Output Hugin pto file.",
    )
    parser.add_argument(
        "-work_dir",
        type=str,
        required=False,
        help="Where all the sci cam images are.",
    )

    parser.add_argument(
        "--no-stitching",
        dest="no_stitching",
        action="store_true",
        help="Generate hugin and optimize only.",
    )

    parser.add_argument(
        "--stitching-only",
        dest="only_stitching",
        action="store_true",
        help="Stitch hugin file only",
    )

    args = parser.parse_args()

    return args


def main():

    args = parse_args()

    # Make a new Panorama object
    p = Panorama()

    if args.output_hugin is None:
        output_hugin = args.bag_name.replace(".bag", ".pto")
    else:
        output_hugin = args.output_hugin

    if not args.only_stitching:
        # Read bagfile
        bag = rosbag.Bag(args.bag_name)
        get_pose = False
        srcImage = SrcPanoImage()
        srcImage.setVar("v", 62)
        for topic, msg, t in bag.read_messages():
            if topic == "/hw/cam_sci/compressed":

                img = (
                    args.work_dir
                    + str(msg.header.stamp.secs)
                    + "."
                    + "%03d" % (msg.header.stamp.nsecs * 0.000001)
                    + ".jpg"
                )
                print(img)
                # Make sure image exists
                if os.path.exists(img) is False:
                    print("Could not find panorama image!!!")
                else:
                    # Insert image into hugin
                    srcImage.setFilename(img)
                    get_pose = True

            if get_pose and topic == "/loc/pose":
                # Configure hugin parameters
                orientation_list = [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                srcImage.setVar("r", roll * RAD2DEG)
                srcImage.setVar("p", pitch * RAD2DEG)
                srcImage.setVar("y", yaw * RAD2DEG)
                p.addImage(srcImage)
                get_pose = False

        bag.close()

        # Link lenses
        variable_groups = StandardImageVariableGroups(p)
        lenses = variable_groups.getLenses()
        for i in range(0, p.getNrOfImages()):
            lenses.switchParts(i, lenses.getPartNumber(0))

        # Write hugin file
        # make a c++ std::ofstream to write to
        ofs = ofstream(output_hugin)
        # write the modified panorama to that stream
        p.writeData(ofs)
        # done with it
        del ofs

        # Generate control points
        cmd = ["cpfind", "--multirow", "-o", output_hugin, output_hugin]
        (returncode, stdout, stderr) = run_cmd(cmd)
        print(stdout)

        # Throw away control points are prob invalid
        cmd = ["cpclean", "-o", output_hugin, output_hugin]
        (returncode, stdout, stderr) = run_cmd(cmd)
        print(stdout)

        # Optimize attitude + b
        cmd = ["pto_var", "--opt", "y,p,r,b", "-o", output_hugin, output_hugin]
        (returncode, stdout, stderr) = run_cmd(cmd)
        cmd = ["autooptimiser", "-n", "-o", output_hugin, output_hugin]
        (returncode, stdout, stderr) = run_cmd(cmd)

        # Optimize position iteratively not to diverge
        ifs = ifstream(output_hugin)  # create a C++ std::ifstream
        p.readData(ifs)  # read the pto file into the Panorama object

        orig_tuple = p.getOptimizeVector()

        pano_size = len(orig_tuple)
        stride = 5
        optim_nr_list = [10, 15]

        for optim_nr in optim_nr_list:

            # Optimizer cycles
            # We do this iteratively otherwise it will diverge
            for x in range(0, pano_size, stride):
                print(x)
                ifs = ifstream(output_hugin)  # create a C++ std::ifstream
                p.readData(ifs)  # read the pto file into the Panorama object
                optvec = []
                for i in range(pano_size):
                    if (x + optim_nr) > pano_size:
                        start = x + optim_nr - pano_size
                    else:
                        start = 0
                    if i >= x and i < x + optim_nr or i < start:
                        optvec.append(
                            ["y", "p", "r", "Tpp", "Tpy", "TrX", "TrY", "TrZ"]
                        )
                    else:
                        optvec.append([])

                print(optvec)
                p.setOptimizeVector(optvec)
                ofs = ofstream(output_hugin)
                p.writeData(ofs)
                del ofs

                cmd = ["autooptimiser", "-n", "-o", output_hugin, output_hugin]
                (returncode, stdout, stderr) = run_cmd(cmd)

            # Optimize attitude + b + v
            cmd = ["pto_var", "--opt", "y,p,r,b,v", "-o", output_hugin, output_hugin]
            (returncode, stdout, stderr) = run_cmd(cmd)
            cmd = ["autooptimiser", "-n", "-o", output_hugin, output_hugin]
            (returncode, stdout, stderr) = run_cmd(cmd)

        # Optimize attitude + position
        cmd = [
            "pto_var",
            "--opt",
            "y,p,r,Tpp,Tpy,TrX,TrY,TrZ",
            "-o",
            output_hugin,
            output_hugin,
        ]
        (returncode, stdout, stderr) = run_cmd(cmd)
        cmd = ["autooptimiser", "-n", "-o", output_hugin, output_hugin]
        (returncode, stdout, stderr) = run_cmd(cmd)

        # Optimize ALL x3
        cmd = [
            "pto_var",
            "--opt",
            "y,p,r,Tpp,Tpy,TrX,TrY,TrZ,b,v",
            "-o",
            output_hugin,
            output_hugin,
        ]
        (returncode, stdout, stderr) = run_cmd(cmd)
        cmd = ["autooptimiser", "-n", "-o", output_hugin, output_hugin]
        (returncode, stdout, stderr) = run_cmd(cmd)
        cmd = [
            "pto_var",
            "--opt",
            "y,p,r,TrX,TrY,TrZ,b,v",
            "-o",
            output_hugin,
            output_hugin,
        ]
        (returncode, stdout, stderr) = run_cmd(cmd)
        cmd = ["autooptimiser", "-n", "-o", output_hugin, output_hugin]
        (returncode, stdout, stderr) = run_cmd(cmd)
        cmd = [
            "pto_var",
            "--opt",
            "y,p,r,Tpp,Tpy,TrX,TrY,TrZ,b,v",
            "-o",
            output_hugin,
            output_hugin,
        ]
        (returncode, stdout, stderr) = run_cmd(cmd)
        cmd = ["autooptimiser", "-n", "-o", output_hugin, output_hugin]
        (returncode, stdout, stderr) = run_cmd(cmd)

        # Photometric Optimizer
        cmd = ["autooptimiser", "-m", "-o", output_hugin, output_hugin]
        (returncode, stdout, stderr) = run_cmd(cmd)

        # Configure image output
        cmd = [
            "pano_modify",
            "-o",
            output_hugin,
            "--center",
            "--canvas=AUTO",
            "--projection=2",
            "--fov=360x180",
            "--output-type=NORMAL,REMAP",
            output_hugin,
        ]
        (returncode, stdout, stderr) = run_cmd(cmd)

    if not args.no_stitching:
        # Generate panorama
        cmd = [
            "hugin_executor",
            "--stitching",
            "--prefix=" + output_hugin,
            output_hugin,
        ]
        (returncode, stdout, stderr) = run_cmd(cmd)


if __name__ == "__main__":

    main()
