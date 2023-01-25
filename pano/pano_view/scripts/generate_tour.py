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
Generate a panoramic tour from stitched and tiled panos.
"""

import argparse
import json
import os

import yaml

PANO_VIEW_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

TOUR_DEFAULT_INIT = {
    "sceneFadeDuration": 1000,
}

TOUR_SCENE_INIT = {
    "title": "ISS {module} {bay}",
    "author": "{author} - captured {date}",
    "type": "multires",
    "hfov": 150.0,
    "autoLoad": True,
}

SCENE_META_INIT = {
    "author": "NASA ISAAC Project",
}


def dosys(cmd, exit_on_error=True):
    print("+ " + cmd)
    ret = os.system(cmd)
    if ret != 0:
        msg = "Command exited with non-zero return value %s" % ret
        if exit_on_error:
            raise RuntimeError(msg)
        print(msg)
    return ret


def install_glob(src_glob, tgt_folder):
    if not os.path.isdir(tgt_folder):
        dosys("mkdir -p %s" % tgt_folder)
    dosys("cp %s %s" % (src_glob, tgt_folder))


def install_static_files(out_folder, pannellum_path):
    install_glob(
        os.path.join(pannellum_path, "build/pannellum.js"),
        os.path.join(out_folder, "js"),
    )
    install_glob(
        os.path.join(pannellum_path, "build/pannellum.css"),
        os.path.join(out_folder, "css"),
    )
    install_glob(
        os.path.join(pannellum_path, "src/standalone/standalone.js"),
        os.path.join(out_folder, "js"),
    )
    install_glob(
        os.path.join(pannellum_path, "src/standalone/standalone.css"),
        os.path.join(out_folder, "css"),
    )
    install_glob(
        os.path.join(PANO_VIEW_ROOT, "static/js/*"), os.path.join(out_folder, "js")
    )
    install_glob(
        os.path.join(PANO_VIEW_ROOT, "static/css/*"), os.path.join(out_folder, "css")
    )
    install_glob(
        os.path.join(PANO_VIEW_ROOT, "media/*"), os.path.join(out_folder, "media")
    )

    # Some HTML files in the templates folder are currently static so
    # we can just install them; in the future we might replace them
    # with templates to provide greater flexibility, which would require
    # template rendering.
    install_glob(os.path.join(PANO_VIEW_ROOT, "templates/pannellum.htm"), out_folder)


def get_display_scene_meta(scene_id, config_scene_meta):
    scene_meta = SCENE_META_INIT.copy()
    scene_meta.update(config_scene_meta)

    scene_meta["scene_id"] = scene_id

    # Replace missing or null field with "" to avoid errors
    for field in ("activity", "module", "bay", "date"):
        if scene_meta.get(field) is None:
            scene_meta[field] = ""

    # Prettify some meta fields for use in templates
    if "activity" in scene_meta:
        scene_meta["activity"] = scene_meta["activity"].upper()
    if "module" in scene_meta:
        scene_meta["module"] = scene_meta["module"].upper()
    if "bay" in scene_meta:
        scene_meta["bay"] = "Bay %s" % scene_meta["bay"]
    if "end_time" in scene_meta:
        scene_meta["date"] = scene_meta["end_time"].split("T", 1)[0]

    return scene_meta


def generate_tour_json(config, out_folder):
    tour_default = TOUR_DEFAULT_INIT.copy()
    tour_default["firstScene"] = next(iter(config["scenes"].keys()))
    tour = {"default": tour_default}

    tour_scenes = {}
    tour["scenes"] = tour_scenes

    for scene_id, config_scene_meta in config["scenes"].items():
        scene_meta = get_display_scene_meta(scene_id, config_scene_meta)

        tour_scene = TOUR_SCENE_INIT.copy()

        # Fill tour_scene fields that are templates
        tour_scene_updates = {}
        for field, val in tour_scene.items():
            if isinstance(val, str) and "{" in val:
                val = val.format(**scene_meta)
                val = val.replace("  ", " ")
                tour_scene_updates[field] = val
        tour_scene.update(tour_scene_updates)

        # Read more scene metadata from the tiler
        tiler_meta_path = os.path.join(out_folder, "scenes", scene_id, "config.json")
        with open(tiler_meta_path, "r") as tiler_meta_stream:
            tiler_meta = json.load(tiler_meta_stream)

        # Copy tiler metadata into scene. Paths output by the tiler
        # start with "/" but are actually relative to the tiler output
        # dir, so we need to strip the leading slash and prepend the
        # path from tour.json to the tiler output dir.
        multi_res_meta = tiler_meta["multiRes"]
        multi_res_meta["path"] = os.path.join(
            "scenes", scene_id, multi_res_meta["path"][1:]
        )
        multi_res_meta["fallbackPath"] = os.path.join(
            "scenes", scene_id, multi_res_meta["fallbackPath"][1:]
        )
        tour_scene["multiRes"] = multi_res_meta

        # Add scene to the tour.
        tour_scenes[scene_id] = tour_scene

    out_path = os.path.join(out_folder, "tour.json")
    with open(out_path, "w") as out:
        json.dump(tour, out, indent=4)
    print("wrote to %s" % out_path)


def generate_scene_index(config, out_folder):
    # Can improve this to be grouped by modules and bays sorted in increasing order.
    index = ["<ul>"]
    for scene_id, config_scene_meta in config["scenes"].items():
        scene_meta = get_display_scene_meta(scene_id, config_scene_meta)
        index.append(
            '<li><a href="pannellum.htm?config=tour.json&firstScene={scene_id}">{module} {bay}</a></li>'.format(
                **scene_meta
            )
        )
    index.append("</ul>")
    index_str = "\n".join(index) + "\n"

    # HACK replace this with a real templating system like Jinja2
    template_path = os.path.join(PANO_VIEW_ROOT, "templates/index.html")
    with open(template_path, "r") as template_stream:
        template = template_stream.read()
    html = template.replace("{{ index }}", index_str)

    out_path = os.path.join(out_folder, "index.html")
    with open(out_path, "w") as out:
        out.write(html)
    print("wrote to %s" % out_path)


def generate_tour(config_path, out_folder, pannellum_path):
    with open(config_path, "r") as config_stream:
        config = yaml.safe_load(config_stream)

    install_static_files(out_folder, pannellum_path)
    generate_tour_json(config, out_folder)
    generate_scene_index(config, out_folder)
    dosys("chmod a+rX %s" % out_folder)


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
        "-o",
        "--out-folder",
        type=str,
        help="output path for static web files of tour",
        default="/output/html",
        required=False,
    )
    parser.add_argument(
        "--pannellum",
        type=str,
        help="path where pannellum source is checked out (will install files from here)",
        default="/opt/pannellum",
        required=False,
    )

    args = parser.parse_args()

    generate_tour(args.config, args.out_folder, args.pannellum)


if __name__ == "__main__":
    main()
