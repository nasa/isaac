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
config file for stitching. Manual review of the config file is
recommended before starting a large stitching job.

If --add is specified, add only newly detected panoramas to the
existing config file, without modifying the existing entries.
"""

import argparse
import copy
import datetime
import os
import random
import re
import sys

import numpy as np
import yaml

import pano_image_meta

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
    print("Detected {} candidate bags".format(len(bags)))
    print("Detected {} candidate SciCam images".format(len(sci_cam_images)))

    scenes = {}
    pano_meta = {"scenes": scenes}
    for i, (bag_path, bag_meta) in enumerate(bags.items()):
        print("Bag {}".format(bag_path))
        print("  {} SciCam images with pose data".format(len(bag_meta)))
        if not bag_meta:
            print("  (Skipping)")
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


def write_pano_meta(pano_meta, out_yaml_path):
    """
    Write pano metadata to a YAML file, to be used as a configfile for
    snakemake.
    """
    with open(out_yaml_path, "w") as out_yaml:
        out_yaml.write(yaml.safe_dump(pano_meta, default_flow_style=False, sort_keys=False))
    print("wrote to %s" % out_yaml_path)


def add_pano_meta(new_meta, out_yaml_path):
    """
    Add new_meta to the existing pano metadata in the YAML file.
    """
    with open(out_yaml_path, "r") as old_yaml_stream:
        old_meta = yaml.safe_load(old_yaml_stream)
    old_bag_paths = set((os.path.realpath(scene["bag_path"])
                         for scene in old_meta["scenes"].values()))

    merged_meta = copy.deepcopy(old_meta)
    scene_prefix = re.compile(r"scene\d\d\d_")
    for scene_id, scene_meta in new_meta["scenes"].items():
        new_bag_path = os.path.realpath(scene_meta["bag_path"])
        if new_bag_path in old_bag_paths:
            continue

        # renumber scene id
        scene_id = scene_prefix.sub("scene%03d_" % len(merged_meta["scenes"]), scene_id)

        merged_meta["scenes"][scene_id] = scene_meta

    num_old = len(old_meta["scenes"])
    num_new = len(new_meta["scenes"])
    num_out = len(merged_meta["scenes"])
    num_added = num_out - num_old
    num_skipped = num_new - num_added

    print("out of %d panos detected, %d were added and %d existing entries were skipped"
          % (num_new, num_added, num_skipped))

    tmp_path = out_yaml_path + ".tmp"
    with open(tmp_path, "w") as out_yaml:
        out_yaml.write(yaml.safe_dump(merged_meta, default_flow_style=False, sort_keys=False))

    stem, suffix = os.path.splitext(out_yaml_path)
    random.seed()
    unique_id = "%0x" % random.getrandbits(32)
    old_yaml_backup_path = "%s-old-%s%s" % (stem, unique_id, suffix)

    os.rename(out_yaml_path, old_yaml_backup_path)
    os.rename(tmp_path, out_yaml_path)

    print("wrote to %s" % out_yaml_path)
    print("old version backed up at %s" % old_yaml_backup_path)


def config_panos(in_folder, out_yaml_path, force, add_panos):
    if os.path.exists(out_yaml_path) and not (force or add_panos):
        print(
            "output file %s exists, not overwriting (did you mean --force or --add?)"
            % out_yaml_path
        )
        sys.exit(1)

    pano_meta = detect_pano_meta(in_folder)
    if os.path.exists(out_yaml_path) and add_panos:
        add_pano_meta(pano_meta, out_yaml_path)
    else:
        write_pano_meta(pano_meta, out_yaml_path)


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
    parser.add_argument(
        "-a",
        "--add",
        action="store_true",
        help="add new panos to existing file without changing old ones",
        default=False,
        required=False,
    )
    args = parser.parse_args()

    config_panos(args.in_folder, args.out_yaml, args.force, args.add)


if __name__ == "__main__":
    main()
