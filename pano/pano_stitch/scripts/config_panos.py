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
Detect panoramas (bag files and associated SciCam images) and write
template config file for stitching.
"""

import argparse
import datetime
import os
import re
import sys

import numpy as np
import pano_image_meta
import yaml

SCI_CAM_IMG_REGEX = re.compile(r"\d{10}\.\d{3}\.jpg$")

ROBOT_REGEX = re.compile(r"(\b|_)(?P<robot>honey|bumble|queen)(\b|_)", re.IGNORECASE)
ACTIVITY_REGEX = re.compile(r"(\b|_)(?P<activity>isaac\d+)(\b|_)", re.IGNORECASE)
MODULE_REGEX = re.compile(
    r"(\b|_)(?P<module>JEM|NOD2|USL|COL|NOD1)(\b|_)", re.IGNORECASE
)
BAY_REGEX = re.compile(r"(\b|_)bay(?P<bay>\d+)(\b|_)", re.IGNORECASE)

SCENE_REGEXES = (
    ("activity", ACTIVITY_REGEX),
    ("robot", ROBOT_REGEX),
    ("module", MODULE_REGEX),
    ("bay", BAY_REGEX),
)

FIELD_PREFIXES = {
    "bay": "bay",
}


def get_scene_position(bag_meta):
    pos_data = np.zeros((len(bag_meta), 3))
    for i, image_meta in enumerate(bag_meta):
        pos_data[i, :] = [image_meta["x"], image_meta["y"], image_meta["z"]]
    median_pos = np.median(pos_data, axis=0)
    return {
        "x": float(median_pos[0]),
        "y": float(median_pos[1]),
        "z": float(median_pos[2]),
    }


def get_image_timestamp(image_meta):
    timestamp = datetime.datetime.utcfromtimestamp(image_meta["timestamp"])
    return timestamp.isoformat() + "Z"


def detect_pano_meta(in_folder):
    """
    Detect panoramas (bag files and associated SciCam images). Return
    pano metadata.
    """

    bags = {}
    sci_cam_images = {}
    for dirname, subdirs, files in os.walk(in_folder):
        for f in files:
            if f.endswith(".bag"):
                bag_path = os.path.join(dirname, f)
                bags[bag_path] = pano_image_meta.get_image_meta(bag_path)
            elif SCI_CAM_IMG_REGEX.search(f):
                sci_cam_images[f] = dirname

    scenes = {}
    pano_meta = {"scenes": scenes}
    for i, (bag_path, bag_meta) in enumerate(bags.items()):
        if not bag_meta:
            # bag_meta would have length 0 if e.g. the bag has no SciCam images
            continue

        scene_meta = {
            "bag_path": bag_path,
            "images_dir": None,
            "robot": None,
            "activity": None,
            "module": None,
            "bay": None,
            "position": get_scene_position(bag_meta),
            "start_time": get_image_timestamp(bag_meta[0]),
            "end_time": get_image_timestamp(bag_meta[-1]),
            "extra_stitch_args": "",
        }
        scene_meta["images_dir"] = sci_cam_images.get(bag_meta[0]["img_path"])

        for field, regex in SCENE_REGEXES:
            match = regex.search(bag_path)
            if match:
                val = match.group(field).lower()
                if val.isdigit():
                    val = int(val)
                scene_meta[field] = val

        scene_id = "scene%03d" % i
        for field, regex in SCENE_REGEXES:
            if field in scene_meta:
                scene_id += "_%s%s" % (FIELD_PREFIXES.get(field, ""), scene_meta[field])

        scenes[scene_id] = scene_meta

    return pano_meta


def write_pano_meta(pano_meta, out_yaml_path, force):
    """
    Write pano metadata to a YAML file, to be used as a configfile for
    snakemake.
    """
    if os.path.exists(out_yaml_path) and not force:
        print(
            "output file %s exists and --force not specified, not overwriting"
            % out_yaml_path
        )
        sys.exit(1)
    with open(out_yaml_path, "w") as out_yaml:
        out_yaml.write(yaml.dump(pano_meta, default_flow_style=False, sort_keys=False))
    print("wrote to %s" % out_yaml_path)


def config_panos(in_folder, out_yaml_path, force):
    pano_meta = detect_pano_meta(in_folder)
    write_pano_meta(pano_meta, out_yaml_path, force)


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    pass


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "-i",
        "--in-folder",
        type=str,
        help="input path for folder to search for bag files and SciCam images",
        default="/input",
        required=False,
    )
    parser.add_argument(
        "-o",
        "--out-yaml",
        type=str,
        help="output path for YAML pano stitch config",
        default="/output/pano_meta.yaml",
        required=False,
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="overwrite output file if it exists",
        default=False,
        required=False,
    )
    args = parser.parse_args()

    config_panos(args.in_folder, args.out_yaml, args.force)


if __name__ == "__main__":
    main()
