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
Prep source images for web viewing (so panorama can link to them).
"""

import argparse
import collections
import functools
import json
import multiprocessing as mp
import operator
import os

import hsi
import yaml

OUTPUT_PNG = False


def dosys(cmd, exit_on_error=True):
    print("+ " + cmd)
    ret = os.system(cmd)
    if ret != 0:
        msg = "Command exited with non-zero return value %s" % ret
        if exit_on_error:
            raise RuntimeError(msg)
        print(msg)
    return ret


def read_pto(pto_path):
    pano = hsi.Panorama()
    ifs = hsi.ifstream(pto_path)
    pano.readData(ifs)
    return pano


def read_scene_source_images_meta(stitch_folder, scene_id, images_dir):
    pto_path = os.path.join(stitch_folder, scene_id, "stitch_final.pto")
    pano = read_pto(pto_path)
    num_images = pano.getNrOfImages()
    images_meta = {}
    for i in range(num_images):
        img = pano.getImage(i)
        img_id = os.path.splitext(os.path.basename(img.getFilename()))[0]
        images_meta[img_id] = {
            "yaw": img.getYaw(),
            "pitch": img.getPitch(),
            "image": os.path.join(images_dir, img_id + ".jpg"),
            "task": "pano",
        }

    return images_meta


def do_prep_image(job_args):
    image_in, dz_out = job_args

    dz_out_parent = os.path.dirname(dz_out)
    if not os.path.exists(dz_out_parent):
        dosys("mkdir -p %s" % dz_out_parent)

    partial_paths = [
        "%s_partial.dzi" % dz_out,
        "%s_partial_files" % dz_out,
    ]
    if any((os.path.exists(p) for p in partial_paths)):
        dosys("rm -rf %s" % (" ".join(partial_paths)))

    png_arg = "--suffix .png" if OUTPUT_PNG else ""
    dosys("vips dzsave %s %s_partial %s" % (image_in, dz_out, png_arg))
    for p in partial_paths:
        dosys("mv %s %s" % (p, p.replace("_partial", "")))

    # Copy original source image to output as well (enables download link)
    ext = os.path.splitext(image_in)[1]
    dosys("cp %s %s" % (image_in, dz_out + ext))


def write_images_meta(images_meta, meta_out_path):
    meta_out_path_parent = os.path.dirname(meta_out_path)
    if not os.path.exists(meta_out_path_parent):
        dosys("mkdir -p %s" % meta_out_path_parent)

    with open(meta_out_path, "w") as meta_out:
        json.dump(images_meta, meta_out, indent=4)
    print("wrote %s" % meta_out_path)


def get_inspection_results(scene_meta):
    infos = scene_meta.get("inspection_results", [])
    for info in infos:
        info["task"] = "inspection"
    return {os.path.splitext(os.path.basename(info["image"]))[0]: info for info in infos}


def get_scene_q(config, stitch_folder, out_folder, scene_id):
    scene_meta = config["scenes"][scene_id]
    images_meta = read_scene_source_images_meta(stitch_folder, scene_id, scene_meta["images_dir"])
    images_meta.update(get_inspection_results(scene_meta))
    scene_out = os.path.join(out_folder, "source_images", scene_id)

    prep_image_q = []
    for img_id, img_meta in images_meta.items():
        image_in = img_meta["image"]
        dz_out = os.path.join(scene_out, img_id)
        if os.path.exists(dz_out + ".dzi"):
            continue
        prep_image_q.append((image_in, dz_out))

    print(
        "%s: %s out of %s source images need prep"
        % (scene_id, len(prep_image_q), len(images_meta))
    )

    meta_out_path = os.path.join(scene_out, "meta.json")
    write_images_meta(images_meta, meta_out_path)

    return prep_image_q


def execute_q(pool, do_job, q):
    result_iterator = pool.imap_unordered(do_job, q)
    # exhaust iterator
    collections.deque(result_iterator, maxlen=0)


def join_lists(lists):
    return functools.reduce(operator.iadd, lists, [])


def prep_source_images(config_path, stitch_folder, out_folder, num_jobs):
    with open(config_path, "r") as config_stream:
        config = yaml.safe_load(config_stream)

    prep_image_q = join_lists(
        (
            get_scene_q(config, stitch_folder, out_folder, scene_id)
            for scene_id in config["scenes"].keys()
        )
    )

    print(
        "queued image prep jobs: %s total jobs for %s scenes"
        % (len(prep_image_q), len(config["scenes"]))
    )

    print("executing job queue")
    with mp.Pool(processes=num_jobs) as pool:
        execute_q(pool, do_prep_image, prep_image_q)

    print(
        "executed prep jobs: %s total jobs for %s scenes"
        % (len(prep_image_q), len(config["scenes"]))
    )


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
        help="input path for YAML pano stitch config",
        default="/output/pano_meta.yaml",
        required=False,
    )
    parser.add_argument(
        "-s",
        "--stitch-folder",
        type=str,
        help="input path for pano stitching results",
        default="/output/stitch",
        required=False,
    )
    parser.add_argument(
        "-o",
        "--out-folder",
        type=str,
        help="output path for static web files",
        default="/output/html",
        required=False,
    )
    parser.add_argument(
        "-j",
        "--jobs",
        type=int,
        help="number of image processing jobs to run in parallel",
        default=mp.cpu_count() // 2,
        required=False,
    )

    args = parser.parse_args()

    prep_source_images(args.config, args.stitch_folder, args.out_folder, args.jobs)


if __name__ == "__main__":
    main()
