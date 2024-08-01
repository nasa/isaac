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
import re

import numpy as np
import scipy.sparse.csgraph
import scipy.spatial.distance
import yaml

PANO_VIEW_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
PACKAGES = [
    "pannellum",
    "openseadragon",
    "annotorious-openseadragon",
    "annotorious-selectorpack",
    "annotorious-toolbar",
]
DEFAULT_PACKAGE_PATHS = {pkg: "/opt/" + pkg for pkg in PACKAGES}

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

SCENE_LINK_HOT_SPOT_TEXT = "{module} {bay}"

# Pick a default starting yaw for each module. Pointing along the centerline
# to start feels more natural.
CENTERLINE_YAW = {
    "jem": 90,
    "nod2": 180,
    "col": 90,
    "usl": 180,
    "nod1": 180,
}

MULTI_SPACE_REGEX = re.compile(r" +")

DEGREES_PER_RADIAN = 180 / np.pi

# This is very roughly calibrated. Note: Overview map may not be exactly to scale!
OVERVIEW_PX_PER_METER = 8.63
OVERVIEW_X0_PX = 118 + OVERVIEW_PX_PER_METER * 1.64
OVERVIEW_Y0_PX = 118


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
    dosys("cp -r %s %s" % (src_glob, tgt_folder))


def install_file(src_path, tgt_folder, tgt_name=None):
    if not os.path.isdir(tgt_folder):
        dosys("mkdir -p %s" % tgt_folder)
    if tgt_name is None:
        tgt_path = tgt_folder
    else:
        tgt_path = os.path.join(tgt_folder, tgt_name)
    dosys("cp -r %s %s" % (src_path, tgt_path))


def install_static_files(out_folder, package_paths):
    pannellum_path = package_paths["pannellum"]
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

    openseadragon_path = package_paths["openseadragon"]
    install_glob(
        os.path.join(openseadragon_path, "openseadragon.min.js*"),
        os.path.join(out_folder, "js"),
    )
    install_glob(
        os.path.join(openseadragon_path, "images/*"),
        os.path.join(out_folder, "media/openseadragon"),
    )

    anno_path = package_paths["annotorious-openseadragon"]
    install_glob(
        os.path.join(anno_path, "*.js"),
        os.path.join(out_folder, "js"),
    )
    install_glob(
        os.path.join(anno_path, "*.js.map"),
        os.path.join(out_folder, "js"),
    )
    install_glob(
        os.path.join(anno_path, "*.css"),
        os.path.join(out_folder, "css"),
    )

    sel_path = package_paths["annotorious-selectorpack"]
    install_glob(
        os.path.join(sel_path, "*.js"),
        os.path.join(out_folder, "js"),
    )
    install_glob(
        os.path.join(sel_path, "*.js.map"),
        os.path.join(out_folder, "js"),
    )

    toolbar_path = package_paths["annotorious-toolbar"]
    install_glob(
        os.path.join(toolbar_path, "annotorious-toolbar.min.js"),
        os.path.join(out_folder, "js"),
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
    install_glob(os.path.join(PANO_VIEW_ROOT, "templates/pannellum.html"), out_folder)
    install_glob(os.path.join(PANO_VIEW_ROOT, "templates/help.html"), out_folder)
    install_file(
        os.path.join(PANO_VIEW_ROOT, "templates/isaac_source_image.html"),
        os.path.join(out_folder, "src"),
        "index.html",
    )


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


def fill_field(tmpl, display_scene_meta):
    # Replace template {patterns} with variable values from display_scene_meta
    val = tmpl.format(**display_scene_meta)
    # Collapse multiple spaces to single space and strip leading and
    # trailing whitespace. (Templates have space-separated fields and
    # some fields may be empty... removing extra spaces looks better.)
    return MULTI_SPACE_REGEX.sub(" ", val).strip()


def get_angles0(p_from, p_to):
    """
    Return yaw and pitch angles that will point a camera at p_from to
    a target at p_to. Both arguments are 3D points.
    """
    d = p_to - p_from
    return {
        "yaw": np.arctan2(d[1], d[0]) * DEGREES_PER_RADIAN,
        "pitch": np.arctan2(d[2], np.sqrt(d[0] ** 2 + d[1] ** 2)) * DEGREES_PER_RADIAN,
    }


def get_angles(p_from, p_to, module_from, force_centerline=False):
    """
    Return yaw and pitch angles that will point a camera at p_from to
    a target at p_to. Both arguments are 3D points. If
    force_centerline is True, a module is defined for p_from, and a
    centerline yaw is defined for that module, force the hot spot
    angles to exactly align with the module centerline in whichever
    direction is nearer to the target yaw.
    """
    angles = get_angles0(p_from, p_to)
    module_yaw = CENTERLINE_YAW.get(module_from)

    if module_yaw is None or not force_centerline:
        return angles

    diff = abs(angles["yaw"] - module_yaw)
    if diff < 180:
        yaw = module_yaw
    else:
        yaw = (module_yaw + 180) % 360
    return {
        "yaw": yaw,
        "pitch": 0,
    }


def get_overview_map_position(scene_meta):
    p = scene_meta["position"]
    x = p["x"]
    y = p["y"]
    return (
        OVERVIEW_Y0_PX + y * OVERVIEW_PX_PER_METER,
        OVERVIEW_X0_PX - x * OVERVIEW_PX_PER_METER,
    )


def link_scenes(config, tour_scenes):
    # Collect positions of scenes
    n = len(config["scenes"])
    pos = np.zeros((n, 3))
    for i, config_scene_meta in enumerate(config["scenes"].values()):
        p = config_scene_meta["position"]
        pos[i, :] = (p["x"], p["y"], p["z"])

    # Calculate Euclidean distance cost matrix M between scenes.
    # M_ij is the Euclidean distance between scene i and scene j.
    cost_matrix = scipy.spatial.distance.cdist(pos, pos, "euclidean")

    # Calculate a minimum spanning tree that spans all scenes.  Each
    # edge in the MST between two scenes will turn into a two-way
    # hot-spot link between the scenes. Because the MST spans all
    # scenes, you should be able to use the links to reach any scene
    # from any other scene. Because we tend to capture panoramas on
    # ISS module centerlines and the centerlines form a tree, with
    # luck using the MST as a heuristic will give us a topology that
    # matches the ISS module topology and will seem natural to users.
    tree = scipy.sparse.csgraph.minimum_spanning_tree(cost_matrix)

    scene_id_lookup = list(config["scenes"].keys())

    for j1, j2 in np.transpose(np.nonzero(tree)):
        for j_from, j_to in ((j1, j2), (j2, j1)):
            scene_id_from = scene_id_lookup[j_from]
            config_scene_meta_from = config["scenes"][scene_id_from]
            tour_scene_from = tour_scenes[scene_id_from]

            scene_id_to = scene_id_lookup[j_to]
            config_scene_meta_to = config["scenes"][scene_id_to]
            scene_meta_to = get_display_scene_meta(scene_id_to, config_scene_meta_to)
            tour_scene_to = tour_scenes[scene_id_to]

            angles = get_angles(
                pos[j_from, :], pos[j_to, :], config_scene_meta_from.get("module")
            )
            hot_spot = {
                "type": "scene",
                "id": scene_id_to,
                "sceneId": scene_id_to,
                "text": fill_field(SCENE_LINK_HOT_SPOT_TEXT, scene_meta_to),
                "yaw": angles["yaw"] - tour_scene_from.get("northOffset", 0),
                "pitch": angles["pitch"],
                "targetYaw": angles["yaw"] - tour_scene_to.get("northOffset", 0),
                "targetPitch": angles["pitch"],
            }

            hot_spots = tour_scene_from.setdefault("hotSpots", [])
            hot_spots.append(hot_spot)


def link_source_images(config, tour_scenes, out_folder):
    for scene_id, config_scene_meta in config["scenes"].items():
        tour_scene = tour_scenes[scene_id]
        hot_spots = tour_scene.setdefault("hotSpots", [])

        src_images_meta_path = os.path.join(
            out_folder, "source_images", scene_id, "meta.json"
        )

        if not os.path.exists(src_images_meta_path):
            print("warning: no source images prepped for scene %s" % scene_id)
            continue

        with open(src_images_meta_path, "r") as src_images_meta_stream:
            src_images_meta = json.load(src_images_meta_stream)

        scene_meta = get_display_scene_meta(scene_id, config_scene_meta)

        img_ids = sorted(src_images_meta.keys())
        for i, img_id in enumerate(img_ids):
            img_num = i + 1
            img_meta = src_images_meta[img_id]
            try:
                # get manually configured slug for inspection result
                text = img_meta["slug"]
                slug = text.replace(" ", "_")
            except KeyError:
                # generic fallback for pano images
                text = "Image %d" % img_num
                slug = "%s_%s_Image_%s" % (
                    scene_meta["module"],
                    scene_meta["bay"],
                    img_num,
                )
            hot_spots.append(
                {
                    "type": "info",
                    "id": img_id,
                    "text": text,
                    "yaw": img_meta["yaw"] - tour_scene.get("northOffset", 0),
                    "pitch": img_meta["pitch"],
                    "task": img_meta["task"],
                    "URL": "src/#scene=%s&imageId=%s&slug=%s"
                    % (scene_id, img_id, slug),
                    "cssClass": f"isaac-source-image isaac-{img_meta['task']} pnlm-hotspot pnlm-sprite",
                    "attributes": {
                        "target": "_blank",
                    },
                }
            )


def generate_tour_json(config, out_folder):
    tour_default = TOUR_DEFAULT_INIT.copy()
    tour_default["firstScene"] = next(iter(config["scenes"].keys()))
    tour = {"default": tour_default}

    tour_scenes = {}
    tour["scenes"] = tour_scenes
    tour["initialAnnotations"] = config["initial_annotations"]

    for scene_id, config_scene_meta in config["scenes"].items():
        # Read tiler scene metadata
        tiler_meta_path = os.path.join(out_folder, "scenes", scene_id, "config.json")
        config_scene_meta["tiled"] = os.path.isfile(tiler_meta_path)
        if not config_scene_meta["tiled"]:
            print("warning: skipping %s (%s not found)" % (scene_id, tiler_meta_path))
            continue
        with open(tiler_meta_path, "r") as tiler_meta_stream:
            tiler_meta = json.load(tiler_meta_stream)

        scene_meta = get_display_scene_meta(scene_id, config_scene_meta)

        tour_scene = TOUR_SCENE_INIT.copy()

        # Fill tour_scene fields that are templates
        tour_scene_updates = {}
        for field, val in tour_scene.items():
            if isinstance(val, str) and "{" in val:
                tour_scene_updates[field] = fill_field(val, scene_meta)
        tour_scene.update(tour_scene_updates)

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

        tour_scene["yaw"] = CENTERLINE_YAW.get(config_scene_meta["module"], 0)
        tour_scene["overviewMapPosition"] = get_overview_map_position(scene_meta)

        extra_tour_params = config_scene_meta.get("extra_tour_params", {})
        tour_scene.update(extra_tour_params)

        # We need to adjust the yaw for northOffset if it is used.
        north_offset = tour_scene.get("northOffset", 0)
        tour_scene["yaw"] -= north_offset

        # Add scene to the tour.
        tour_scenes[scene_id] = tour_scene

    link_scenes(config, tour_scenes)
    link_source_images(config, tour_scenes, out_folder)

    out_path = os.path.join(out_folder, "tour.json")
    with open(out_path, "w") as out:
        json.dump(tour, out, indent=4)
    print("wrote to %s" % out_path)


def generate_scene_index(config, out_folder):
    # Can improve this to be grouped by modules and bays sorted in increasing order.
    index = ["<ul>"]
    for scene_id, config_scene_meta in config["scenes"].items():
        if not config_scene_meta["tiled"]:
            continue

        scene_meta = get_display_scene_meta(scene_id, config_scene_meta)
        index.append(
            fill_field(
                (
                    '<li><a href="pannellum.html#config=tour.json&firstScene={scene_id}">'
                    "{module} {bay}"
                    "</a></li>"
                ),
                scene_meta,
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


def install_pano_images(config, out_folder):
    for scene_id, config_scene_meta in config["scenes"].items():
        # Would be more in the spirit of things to not hard-code this input path
        in_image = os.path.join("/output/stitch", scene_id, "pano.jpg")
        out_image_folder = os.path.join(out_folder, "scenes", scene_id)
        install_file(in_image, out_image_folder, "pano.jpg")


def reorganize_config(config):
    """
    Modify `config` in place. For the top-level inspection_results
    field: add the value for each scene into the config field of the
    same name for that scene. (And delete the top-level field.)
    """
    scenes = config["scenes"]
    for field in ["inspection_results"]:
        value = config.pop(field, {})
        for scene_id, scene_value in value.items():
            scenes[scene_id][field] = scene_value


def generate_tour(config_path, out_folder, package_paths):
    with open(config_path, "r") as config_stream:
        config = yaml.safe_load(config_stream)
    reorganize_config(config)

    install_static_files(out_folder, package_paths)
    generate_tour_json(config, out_folder)
    generate_scene_index(config, out_folder)
    install_pano_images(config, out_folder)
    dosys("chmod -R a+rX %s" % out_folder)


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
        "--package-paths",
        type=str,
        help="comma-separated list of package paths to override formatted like 'openseadragon=/opt/openseadragon'",
        default=None,
        required=False,
    )

    args = parser.parse_args()

    package_paths = DEFAULT_PACKAGE_PATHS.copy()
    if args.package_paths is not None:
        updates = [assn.split("=", 1) for assn in args.package_paths.split(",")]
        package_paths.update(dict(updates))
    generate_tour(args.config, args.out_folder, package_paths)


if __name__ == "__main__":
    main()
