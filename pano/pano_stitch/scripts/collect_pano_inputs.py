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
Example script for collecting, fixing, and filtering pano input data
prior to transferring it to another host for stitching. The relevant
bags and images are specifed in a CSV config file.  As the bags are
collected, some CSV metadata will be embedded in the folder names so
config_panos.py should be able to detect it.

This script is designed to be run in a Vagrant box on the hivemind
server where "hivemind:/home/p-astrobee/webdir/testsessions" is
mounted in the Vagrant box at "/testsessions".
"""

import argparse
import csv
import glob
import os
import sys

import pano_image_meta

PANO_STITCH_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DEFAULT_CONFIG = os.path.join(PANO_STITCH_ROOT, "config", "isaac_phase1x_panos.csv")
FILTER_ARGS = "-a /hw/cam_sci/compressed -a /loc/pose"


def dosys(cmd, exit_on_error=True, echo=True):
    if echo:
        print("+ " + cmd)
    ret = os.system(cmd)
    if ret != 0:
        msg = "Command exited with non-zero return value %s" % ret
        if exit_on_error:
            raise RuntimeError(msg)
        print(msg)
    return ret


def get_image_path(images_dir, img_name):
    img_path = os.path.join(images_dir, img_name)
    if os.path.exists(img_path):
        return img_path

    # special case fallback for ISAAC6 pano with weird sci_cam_image filenames
    sec, subsec, ext = img_name.split(".")
    img_glob = os.path.join(images_dir, "%s.*.%s" % (sec, ext))
    candidates = glob.glob(img_glob)
    if len(candidates) == 0:
        return img_path

    use_path = candidates[0]
    if len(candidates) > 1:
        print("warning: multiple images match %s" % img_glob)
        print("  arbitrarily using first match %s" % use_path)
    return use_path


def collect_pano_inputs(config_path, out_path, num_jobs):
    # add provenance info
    if not os.path.exists(out_path):
        dosys("mkdir -p %s" % out_path)
    dosys("cp %s %s" % (config_path, out_path))
    readme_path = os.path.join(out_path, "README.txt")
    with open(readme_path, "w") as readme:
        readme.write("Panorama inputs collected by collect_pano_inputs.py\n")
        readme.write("Command was: %s\n" % sys.argv)
        readme.write("See config file: %s" % os.path.basename(config_path))

    with open(config_path, "r") as config_stream:
        panos = list(csv.DictReader(config_stream))

    for pano in panos:
        pano["sub_out"] = "%s/%s_%s" % (out_path, pano["activity"], pano["robot"])
        pano["out_bag"] = os.path.join(
            pano["sub_out"], os.path.basename(pano["bag_path"])
        )
        pano["out_bag_partial"] = os.path.join(
            pano["sub_out"], "partial-" + os.path.basename(pano["bag_path"])
        )
        pano["out_bag_fix_all"] = (
            os.path.splitext(pano["out_bag_partial"])[0] + ".fix_all.bag"
        )

        if os.path.exists(pano["out_bag"]):
            print("filtered bag %s exists, not overwriting" % pano["out_bag"])
            continue

        if not os.path.isdir(pano["sub_out"]):
            dosys("mkdir -p %s" % pano["sub_out"])

    # First filtering directly with rosbag_topic_filter.py. This should work
    # if the bag was already fixed.
    for pano in panos:
        if os.path.exists(pano["out_bag"]):
            continue
        ret = dosys(
            "rosrun bag_processing rosbag_topic_filter.py %s %s -o %s"
            % (pano["bag_path"], FILTER_ARGS, pano["out_bag_partial"]),
            exit_on_error=False,
        )
        if ret == 0:
            dosys("mv %s %s" % (pano["out_bag_partial"], pano["out_bag"]))
        else:
            print(
                "%s: simple bag filtering failed - will try to fix and filter this bag later"
                % pano["bag_path"]
            )
            print()

    ######################################################################
    # Fix and filter (only applied if simple filter didn't work)

    # Fix and filter step 1 - symlink
    for pano in panos:
        if os.path.exists(pano["out_bag"]):
            continue
        # Make a (symlink) temporary copy of the input bag in the
        # desired output folder because we can't otherwise control the
        # rosbag_fix_all.py output path. Use a symlink to avoid
        # unnecessarily copying the large bag file.
        dosys("ln -sf %s %s" % (pano["bag_path"], pano["out_bag_partial"]))

    # Fix and filter step 2 - rosbag_fix_all.py (runs in parallel)
    bags_to_filter = [
        pano["out_bag_partial"] for pano in panos if not os.path.exists(pano["out_bag"])
    ]
    if bags_to_filter:
        dosys(
            "rosrun bag_processing rosbag_fix_all.py --filter='%s' -j%s --no-merge %s"
            % (FILTER_ARGS, num_jobs, " ".join(bags_to_filter))
        )

    # Fix and filter step 3 - Delete temp symlinks and rename fixed bags
    for pano in panos:
        if os.path.exists(pano["out_bag"]):
            continue

        dosys("rm %s" % pano["out_bag_partial"])
        dosys("mv %s %s" % (pano["out_bag_fix_all"], pano["out_bag"]))

    # Copy images referenced by bags
    for pano in panos:
        images_dir = os.path.realpath(
            os.path.join(
                os.path.dirname(pano["bag_path"]), pano["images_dir_rel_bag_path"]
            )
        )

        out_images_dir = os.path.join(pano["sub_out"], os.path.basename(images_dir))
        if not os.path.isdir(out_images_dir):
            dosys("mkdir -p %s" % out_images_dir)

        images = pano_image_meta.get_image_meta(pano["out_bag"])
        copied = 0
        missing = 0
        for img in images:
            ret = dosys(
                "cp -n %s %s"
                % (
                    get_image_path(images_dir, img["img_path"]),
                    os.path.join(out_images_dir, img["img_path"]),
                ),
                echo=False,
                exit_on_error=False,
            )
            if ret == 0:
                copied += 1
            else:
                missing += 1
        print(
            "Images for bag %s: %s copied, %s missing"
            % (pano["bag_path"], copied, missing)
        )

    # Collection complete. Tell the user how to sync to the remote host.
    print()
    print(
        "Would run the following transfer command, but due to security rules you may\n"
        "need to initiate the rsync from the other side:"
    )
    print("  rsync -rz --info=progress2 %s <remote_path>" % out_path)


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    pass


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        help="input path for CSV listing of pano inputs",
        default=DEFAULT_CONFIG,
        required=False,
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="where to output collected pano stitching inputs",
        default="/shared/collect_pano_inputs_{}".format(os.environ["USER"]),
        required=False,
    )
    parser.add_argument(
        "-j",
        "--jobs",
        type=int,
        help="number of jobs to run in parallel (passed to rosbag_fix_all.py)",
        default=4,
        required=False,
    )

    args = parser.parse_args()

    collect_pano_inputs(args.config, args.output, args.jobs)


if __name__ == "__main__":
    main()
