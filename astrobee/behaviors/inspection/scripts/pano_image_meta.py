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
Extract pano image poses and write to CSV for convenient debug plotting.
"""

import argparse
import csv

import rosbag
from tf.transformations import euler_from_quaternion

IMAGE_TOPIC = "/hw/cam_sci/compressed"
POSE_TOPIC = "/loc/pose"
FIELD_NAMES = (
    "timestamp",
    "img_path",
    "x",
    "y",
    "z",
    "roll",
    "pitch",
    "yaw",
)


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    pass


def get_image_meta(inbag_path):
    images = []
    with rosbag.Bag(inbag_path) as bag:
        img_meta = None
        for topic, msg, t in bag.read_messages([IMAGE_TOPIC, POSE_TOPIC]):
            if topic == IMAGE_TOPIC:
                img_meta = {}
                images.append(img_meta)

                # fill meta from image message
                img_meta["timestamp"] = (
                    msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
                )
                img_meta["img_path"] = "%d.%03d.jpg" % (
                    msg.header.stamp.secs,
                    msg.header.stamp.nsecs * 1e-6,
                )

            if img_meta is not None and topic == POSE_TOPIC:
                # fill meta from the next pose message after the image message
                img_meta["x"] = msg.pose.position.x
                img_meta["y"] = msg.pose.position.y
                img_meta["z"] = msg.pose.position.z

                orientation_list = [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                img_meta["roll"] = roll
                img_meta["pitch"] = pitch
                img_meta["yaw"] = yaw

                img_meta = None

    return images


def pano_image_meta(in_bag, out_csv):
    images = get_image_meta(in_bag)
    with open(out_csv, "w") as out:
        writer = csv.DictWriter(out, fieldnames=FIELD_NAMES)
        writer.writeheader()
        for img_meta in images:
            writer.writerow(img_meta)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "in_bag",
        type=str,
        help="Input bagfile containing SciCam images and pose messages.",
    )
    parser.add_argument(
        "out_csv",
        type=str,
        help="Output CSV with metadata",
    )
    args = parser.parse_args()

    pano_image_meta(args.in_bag, args.out_csv)


if __name__ == "__main__":
    main()
