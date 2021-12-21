#!/usr/bin/env python

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

"""
A wrapper around the tools that when run together produce a textured mesh.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys

import cv2


def process_args(args):
    """
    Set up the parser and parse the args.
    """

    # Extract some paths before the args are parsed
    src_path = os.path.dirname(args[0])
    exec_path = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(src_path)))
    )
    exec_path = os.path.join(exec_path, "devel/lib/geometry_mapper")

    astrobee_build_dir = os.environ["ASTROBEE_BUILD_PATH"]

    parser = argparse.ArgumentParser(description="Parameters for the geometry mapper.")
    parser.add_argument(
        "--ros_bag",
        dest="ros_bag",
        default="",
        help="A ROS bag with recorded image and point cloud data.",
    )
    parser.add_argument(
        "--sparse_map",
        dest="sparse_map",
        default="",
        help="A registered sparse map made with some of the ROS bag data.",
    )
    parser.add_argument(
        "--output_dir",
        dest="output_dir",
        default="",
        help="The directory where to write the processed data.",
    )
    parser.add_argument(
        "--camera_types",
        dest="camera_types",
        default="sci_cam nav_cam haz_cam",
        help="Specify the cameras to use for the textures, as a list in quotes.",
    )
    parser.add_argument(
        "--camera_topics",
        dest="camera_topics",
        default="/hw/cam_sci/compressed /mgt/img_sampler/nav_cam/image_record /hw/depth_haz/extended/amplitude_int",
        help="Specify the bag topics for the cameras to texture (in the same order as in "
        + "--camera_types). Use a list in quotes.",
    )
    parser.add_argument(
        "--undistorted_crop_wins",
        dest="undistorted_crop_wins",
        default="sci_cam,1250,1000 nav_cam,1100,776 haz_cam,210,160",
        help="The central region to keep after undistorting an image and "
        + "before texturing. For sci_cam the numbers are at 1/4th of the full "
        + "resolution and will be adjusted for the actual input image dimensions. "
        + "Use a list in quotes.",
    )
    parser.add_argument(
        "--haz_cam_points_topic",
        dest="haz_cam_points_topic",
        default="/hw/depth_haz/points",
        help="The depth point cloud topic in the bag file.",
    )
    parser.add_argument(
        "--start",
        dest="start",
        default="0.0",
        help="How many seconds into the bag to start processing the data.",
    )
    parser.add_argument(
        "--duration",
        dest="duration",
        default="-1.0",
        help="For how many seconds to do the processing.",
    )
    parser.add_argument(
        "--sampling_spacing_seconds",
        dest="sampling_spacing_seconds",
        default="2",
        help="Spacing to use, in seconds, between consecutive depth images in "
        "the bag that are processed.",
    )
    parser.add_argument(
        "--dist_between_processed_cams",
        dest="dist_between_processed_cams",
        default="0.1",
        help="Once an image or depth image is processed, how far the camera "
        + "should move (in meters) before it should process more data.",
    )
    parser.add_argument(
        "--sci_cam_timestamps",
        dest="sci_cam_timestamps",
        default="",
        help="Process only these sci cam timestamps (rather than "
        + "any in the bag using --dist_between_processed_cams, etc.). "
        + "Must be a file with one timestamp per line.",
    )
    parser.add_argument(
        "--depth_exclude_columns",
        dest="depth_exclude_columns",
        default="0",
        help="Remove this many columns of data from the rectangular "
        + "depth image sensor at margins to avoid distortion.",
    )
    parser.add_argument(
        "--depth_exclude_rows",
        dest="depth_exclude_rows",
        default="0",
        help="Remove this many rows of data from the rectangular "
        + "depth image sensor at margins to avoid distortion.",
    )
    parser.add_argument(
        "--foreshortening_delta",
        dest="foreshortening_delta",
        default="5.0",
        help="A smaller value here will result in holes in depth images "
        + "being filled more aggressively but potentially with more artifacts "
        + "in foreshortened regions.",
    )
    parser.add_argument(
        "--median_filters",
        dest="median_filters",
        default="7 0.1 25 0.1",
        help='Given a list "w1 d1 w2 d2 ... ", remove a depth image point '
        + "if it differs, in the Manhattan norm, from the median of depth points "
        + "in the pixel window of size wi centered at it by more than di. This "
        + "removes points sticking out for each such i.",
    )
    parser.add_argument(
        "--depth_hole_fill_diameter",
        dest="depth_hole_fill_diameter",
        default="30",
        help="Fill holes in the depth point clouds with this diameter, in pixels. This happens before the clouds are fused. It is suggested to not make this too big, as more hole-filling happens on the fused mesh later (--max_hole_diameter).",
    )
    parser.add_argument(
        "--reliability_weight_exponent",
        dest="reliability_weight_exponent",
        default="2",
        help="A larger value will give more weight to depth points corresponding to pixels closer to depth image center, which are considered more reliable.",
    )
    parser.add_argument(
        "--max_ray_length",
        dest="max_ray_length",
        default="2.0",
        help="Process haz cam depth image points no further than "
        + "this distance from the camera.",
    )
    parser.add_argument(
        "--voxel_size",
        dest="voxel_size",
        default="0.01",
        help="When fusing the depth point clouds use a voxel of this size, "
        + "in meters.",
    )
    parser.add_argument(
        "--voxblox_integrator",
        dest="voxblox_integrator",
        default="merged",
        help="When fusing the depth point clouds use this VoxBlox method. "
        + 'Options are: "merged", "simple", and "fast".',
    )
    parser.add_argument(
        "--max_iso_times_exposure",
        dest="max_iso_times_exposure",
        default="5.1",
        help="Apply the inverse gamma transform to images, multiply them by "
        + "max_iso_times_exposure/ISO/exposure_time to adjust for "
        + "lightning differences, then apply the gamma transform back. "
        + "This value should be set to the maximum observed "
        + "ISO * exposure_time. The default is 5.1. Not used with simulated data.",
    )
    parser.add_argument(
        "--smoothing_time",
        dest="smoothing_time",
        default="0.00005",
        help="A larger value will result in a smoother mesh.",
    )
    parser.add_argument(
        "--max_num_hole_edges",
        dest="max_num_hole_edges",
        default="1000",
        help="Close holes in the mesh which have no more than this many edges.",
    )
    parser.add_argument(
        "--max_hole_diameter",
        dest="max_hole_diameter",
        default="0.3",
        help="The diameter (in meters) of the largest hole in the mesh to fill.",
    )
    parser.add_argument(
        "--num_min_faces_in_component",
        dest="num_min_faces_in_component",
        default="100",
        help="Keep only connected mesh components with at least this many faces.",
    )
    parser.add_argument(
        "--num_components_to_keep",
        dest="num_components_to_keep",
        default="10",
        help="How many of the largest connected components "
        + "of the mesh to keep. Being too aggressive here can result in a mesh "
        + "with missing parts.",
    )
    parser.add_argument(
        "--edge_keep_ratio",
        dest="edge_keep_ratio",
        default="0.2",
        help="Simply the mesh keeping only this fraction of the original edges.",
    )
    parser.add_argument(
        "--merge_maps",
        dest="merge_maps",
        default="",
        help="Given several output geometry mapper directories, specified "
        + "as a list in quotes, create a merged textured mesh. The input "
        + "bag and sparse map will not be used. Each input geometry mapper run "
        + "can have its own bag and sparse map. The sparse maps must be "
        + "registered to a global coordinate system and co-registered to "
        + "each other, such as when extracted from a larger merged and "
        + "registered map.",
    )
    parser.add_argument(
        "--start_step",
        dest="start_step",
        default="0",
        help="Start processing at this step. Useful for resuming work. "
        + "See the doc for options.",
    )
    parser.add_argument(
        "--astrobee_build_dir",
        dest="astrobee_build_dir",
        default=astrobee_build_dir,
        help="The path to the Astrobee build directory.",
    )
    parser.add_argument(
        "--localization_options",
        dest="localization_options",
        default="--min_surf_features 400 --max_surf_features 600 --min_surf_threshold 5 --default_surf_threshold 10 --max_surf_threshold 1000 --early_break_landmarks 400 --verbose_localization",
        help="Options to to use to localize the nav cam images.",
    )

    parser.add_argument(
        "--use_brisk_map",
        dest="use_brisk_map",
        action="store_true",
        help="Instead of a SURF sparse map made from the same bag that "
        + " needs texturing, use a pre-existing and unrelated BRISK map. "
        + "This map may be more convenient but less reliable.",
    )
    parser.add_argument(
        "--simulated_data",
        dest="simulated_data",
        action="store_true",
        help="If specified, use data recorded in simulation. "
        + "Then haz and sci camera poses and intrinsics should be recorded "
        + "in the bag file.",
    )
    parser.add_argument(
        "--external_mesh",
        dest="external_mesh",
        default="",
        help="Use this mesh to texture the images, rather than creating one "
        + "from depth data in the current bag.",
    )
    parser.add_argument(
        "--nav_cam_to_sci_cam_offset_override_value",
        dest="nav_cam_to_sci_cam_offset_override_value",
        default="",
        help="Override the value of nav_cam_to_sci_cam_timestamp_offset "
        + "from the robot config file with this value.",
    )
    parser.add_argument(
        "--verbose",
        dest="verbose",
        action="store_true",
        help="Echo all output in the terminal.",
    )
    parser.add_argument(
        "--save_debug_data",
        dest="save_debug_data",
        action="store_true",
        help="Save many intermediate datasets for debugging.",
    )
    args = parser.parse_args()

    # Parse the crop windows
    crop_wins = args.undistorted_crop_wins.split()
    crop_win_map = {}
    for crop_win in crop_wins:
        vals = crop_win.split(",")
        if len(vals) != 3:
            raise Exception(
                "Invalid value for --undistorted_crop_wins: "
                + args.undistorted_crop_wins
            )
        crop_win_map[vals[0]] = vals[1:]

    return (src_path, exec_path, crop_win_map, args)


def sanity_checks(geometry_mapper_path, batch_tsdf_path, crop_win_map, args):

    # Check if the environment was set
    for var in [
        "ASTROBEE_RESOURCE_DIR",
        "ASTROBEE_CONFIG_DIR",
        "ASTROBEE_WORLD",
        "ASTROBEE_ROBOT",
    ]:
        if var not in os.environ:
            raise Exception("Must set " + var)

    if not os.path.exists(geometry_mapper_path):
        raise Exception("Cannot find the geometry mapper: " + geometry_mapper_path)

    if not os.path.exists(batch_tsdf_path):
        raise Exception("Cannot find batch_tsdf (a voxblox tool): " + batch_tsdf_path)

    if not os.path.isdir(args.astrobee_build_dir):
        raise Exception(
            "Cannot find the astrobee_build directory. "
            + "Specify it via --astrobee_build_dir."
        )

    camera_types = args.camera_types.split()

    if args.output_dir == "":
        raise Exception("The path to the output directory was not specified.")

    if len(camera_types) != len(args.camera_topics.split()):
        raise Exception("There must be as many camera types as camera topics.")

    if (not args.simulated_data) and len(camera_types) != len(
        args.undistorted_crop_wins.split()
    ):
        raise Exception(
            "There must be as many camera types as listed undistorted "
            + "crop windows."
        )

    if args.simulated_data and "nav_cam" in camera_types:
        raise Exception(
            "The geometry mapper does not support nav_cam with simulated data as "
            + "its distortion is not modeled."
        )

    if args.simulated_data and "haz_cam" not in camera_types:
        raise Exception(
            "The haz_cam must be one of the camera types in simulation mode "
            + "as it is needed to read the simulated camera pose in order to "
            + "process the depth clouds."
        )

    if not args.simulated_data:
        for cam in camera_types:
            if not (cam in crop_win_map):
                raise Exception(
                    "No crop win specified in --undistorted_crop_wins for camera: "
                    + cam
                )


def mkdir_p(path):
    if path == "":
        return  # this can happen when path is os.path.dirname("myfile.txt")
    try:
        os.makedirs(path)
    except OSError:
        if os.path.isdir(path):
            pass
        else:
            raise Exception(
                "Could not make directory " + path + " as a file with this name exists."
            )


def setup_outputs(args):
    mkdir_p(args.output_dir)


def format_cmd(cmd):
    """If some command arguments have spaces, quote them. Then concatenate the results."""
    ans = ""
    for val in cmd:
        if " " in val or "\t" in cmd:
            val = '"' + val + '"'
        ans += val + " "
    return ans


def run_cmd(cmd, log_file, verbose=False):
    """
    Run a command and write the output to a file. In verbose mode also print to screen.
    """

    cmd_str = format_cmd(cmd)
    print(cmd_str + "\n")

    with open(log_file, "w", buffering=0) as f:  # replace 'w' with 'wb' for Python 3
        f.write(cmd_str + "\n")
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        for line in iter(
            process.stdout.readline, ""
        ):  # replace '' with b'' for Python 3
            if verbose:
                sys.stdout.write(line)
            f.write(line)

        # If a certain step failed, do not continue
        process.wait()
        if process.returncode != 0:
            print("Failed execution of: " + " ".join(cmd))
            sys.exit(1)


def compute_poses_and_clouds(geometry_mapper_path, args):
    """
    Invoke the geometry_mapper tool to compute needed camera poses and clouds.
    """

    cmd = [
        geometry_mapper_path,
        "--ros_bag",
        args.ros_bag,
        "--output_dir",
        args.output_dir,
        "--camera_topics",
        args.camera_topics,
        "--haz_cam_points_topic",
        args.haz_cam_points_topic,
        "--camera_types",
        args.camera_types,
        "--start",
        args.start,
        "--duration",
        args.duration,
        "--sampling_spacing_seconds",
        args.sampling_spacing_seconds,
        "--dist_between_processed_cams",
        args.dist_between_processed_cams,
        "--max_iso_times_exposure",
        args.max_iso_times_exposure,
        "--depth_exclude_columns",
        args.depth_exclude_columns,
        "--depth_exclude_rows",
        args.depth_exclude_rows,
        "--foreshortening_delta",
        args.foreshortening_delta,
        "--depth_hole_fill_diameter",
        args.depth_hole_fill_diameter,
        "--reliability_weight_exponent",
        args.reliability_weight_exponent,
        "--median_filters",
        args.median_filters,
    ] + args.localization_options.split(" ")

    if args.sci_cam_timestamps != "":
        cmd += ["--sci_cam_timestamps", args.sci_cam_timestamps]

    if args.use_brisk_map:
        cmd += ["--use_brisk_map"]

    if args.simulated_data:
        cmd += ["--simulated_data"]
    else:
        cmd += ["--sparse_map", args.sparse_map]

    if args.save_debug_data:
        cmd += ["--save_debug_data"]

    if args.nav_cam_to_sci_cam_offset_override_value != "":
        cmd += [
            "--nav_cam_to_sci_cam_offset_override_value",
            args.nav_cam_to_sci_cam_offset_override_value,
        ]

    log_file = os.path.join(args.output_dir, "geometry_mapper_log.txt")
    print(
        "Compute camera poses and extract data from the bag. Writing the output log to: "
        + log_file
    )
    run_cmd(cmd, log_file, verbose=args.verbose)


def fuse_clouds(batch_tsdf_path, mesh, args):
    """
    Invoke the voxblox batch_tsdf tool to fuse the depth images.
    """

    depth_cam_index = os.path.join(args.output_dir, "depth_cam_index.txt")

    cmd = [
        batch_tsdf_path,
        depth_cam_index,
        mesh,
        args.max_ray_length,
        args.voxel_size,
        args.voxblox_integrator,
    ]

    log_file = os.path.join(args.output_dir, "voxblox_log.txt")
    print(
        "Fusing the depth images with voxblox. Writing the output log to: " + log_file
    )
    run_cmd(cmd, log_file, verbose=args.verbose)


def smoothe_mesh(input_mesh, output_mesh, args, attempt):

    smoothe_mesh_path = os.path.join(
        os.environ["HOME"], "projects/cgal_tools/smoothe_mesh"
    )
    if not os.path.exists(smoothe_mesh_path):
        raise Exception("Cannot find the smoothing tool:" + smoothe_mesh_path)

    num_iterations = "1"
    smoothe_boundary = "1"
    cmd = [
        smoothe_mesh_path,
        num_iterations,
        args.smoothing_time,
        smoothe_boundary,
        input_mesh,
        output_mesh,
    ]

    log_file = os.path.join(
        args.output_dir, "smooth_mesh_attempt_" + str(attempt) + "_log.txt"
    )
    print("Smoothing the mesh. Writing the output log to: " + log_file)
    run_cmd(cmd, log_file, verbose=args.verbose)


def fill_holes_in_mesh(input_mesh, output_mesh, args, attempt):

    fill_holes_path = os.path.join(os.environ["HOME"], "projects/cgal_tools/fill_holes")
    if not os.path.exists(fill_holes_path):
        raise Exception("Cannot find the hole-filling tool:" + fill_holes_path)

    cmd = [
        fill_holes_path,
        args.max_hole_diameter,
        args.max_num_hole_edges,
        input_mesh,
        output_mesh,
    ]

    log_file = os.path.join(
        args.output_dir, "fill_holes_attempt_" + str(attempt) + "_log.txt"
    )
    print("Hole-filling the mesh. Writing the output log to: " + log_file)
    run_cmd(cmd, log_file, verbose=args.verbose)


def rm_connected_components(input_mesh, output_mesh, args):

    rm_connected_components_path = os.path.join(
        os.environ["HOME"], "projects/cgal_tools/rm_connected_components"
    )
    if not os.path.exists(rm_connected_components_path):
        raise Exception("Cannot find the tool:" + rm_connected_components_path)

    cmd = [
        rm_connected_components_path,
        args.num_components_to_keep,
        args.num_min_faces_in_component,
        input_mesh,
        output_mesh,
    ]

    log_file = os.path.join(args.output_dir, "rm_connected_components_log.txt")
    print(
        "Removing small connected components from mesh. Writing the output log to: "
        + log_file
    )
    run_cmd(cmd, log_file, verbose=args.verbose)


def simplify_mesh(input_mesh, output_mesh, args):

    simplify_mesh_path = os.path.join(
        os.environ["HOME"], "projects/cgal_tools/simplify_mesh"
    )
    if not os.path.exists(simplify_mesh_path):
        raise Exception("Cannot find the tool:" + simplify_mesh_path)

    cmd = [simplify_mesh_path, args.edge_keep_ratio, input_mesh, output_mesh]

    log_file = os.path.join(args.output_dir, "simplify_mesh_log.txt")
    print("Simplifying the mesh. Writing the output log to: " + log_file)
    run_cmd(cmd, log_file, verbose=args.verbose)


def undistort_images(
    args, cam_type, dist_image_list, undist_dir, undist_crop_win, scale
):

    undistort_image_path = os.path.join(
        args.astrobee_build_dir, "devel/lib/camera/undistort_image"
    )
    if not os.path.exists(undistort_image_path):
        raise Exception("Cannot find the undistort_image tool: " + undistort_image_path)

    cmd = [
        undistort_image_path,
        "--undistorted_crop_win",
        str(undist_crop_win[0]) + " " + str(undist_crop_win[1]),
        "--save_bgr",
        "--robot_camera",
        cam_type,
        "-image_list",
        dist_image_list,
        "--scale",
        str(scale),
        "--output_directory",
        undist_dir,
    ]

    log_file = os.path.join(args.output_dir, "undist_" + cam_type + "_log.txt")
    print(
        "Undistorting " + cam_type + " images. Writing the output log to: " + log_file
    )
    run_cmd(cmd, log_file, verbose=args.verbose)


def create_texrecon_cameras(args, src_path, undist_dir, cam_type):

    cam_to_texrecon_path = os.path.join(src_path, "cameras_to_texrecon.py")
    if not os.path.exists(cam_to_texrecon_path):
        raise Exception("Cannot find: " + cam_to_texrecon_path)

    cmd = [
        "python",
        cam_to_texrecon_path,
        "--camera_dir",
        args.output_dir,
        "--undistorted_image_dir",
        undist_dir,
        "--camera_type",
        cam_type,
    ]

    log_file = os.path.join(args.output_dir, "setup_texrecon_" + cam_type + "_log.txt")
    print(
        "Preparing texrecon cameras for "
        + cam_type
        + ". Writing the output log to: "
        + log_file
    )
    run_cmd(cmd, log_file, verbose=args.verbose)


def run_texrecon(args, src_path, mesh, undist_dir, cam_type):

    # That is one long path
    texrecon_path = os.path.dirname(os.path.dirname(os.path.dirname(exec_path)))
    texrecon_path = os.path.join(
        texrecon_path,
        "build/geometry_mapper/texrecon/src/texrecon-build",
        "apps/texrecon/texrecon",
    )
    if not os.path.exists(texrecon_path):
        raise Exception("Cannot find: " + texrecon_path)

    texrecon_dir = os.path.join(args.output_dir, cam_type + "_texture/run")
    parent_dir = os.path.dirname(texrecon_dir)
    if os.path.isdir(parent_dir):
        # Wipe the existing directory
        print("Removing recursively old directory: " + parent_dir)
        shutil.rmtree(parent_dir)
    mkdir_p(texrecon_dir)

    cmd = [
        texrecon_path,
        undist_dir,
        mesh,
        texrecon_dir,
        "-o",
        "gauss_clamping",
        "-d",
        "view_dir_dot_face_dir",
        "--keep_unseen_faces",
    ]

    log_file = os.path.join(args.output_dir, "texrecon_" + cam_type + "_log.txt")
    print(
        "Running texrecon for " + cam_type + ". Writing the output log to: " + log_file
    )
    run_cmd(cmd, log_file, verbose=args.verbose)

    textured_mesh = texrecon_dir + ".obj"
    return textured_mesh


# Find the ratio of a sci cam image's width and the width from the camera config file.
def find_sci_cam_scale(image_file):
    # width from the imag file
    img = cv2.imread(image_file)
    image_width = img.shape[1]

    # Width from the config file
    config_file = os.environ["ASTROBEE_CONFIG_DIR"] + "/cameras.config"

    with open(config_file, "r") as file:
        text = file.read()
        m = re.match("^.*?sci_cam\s*=\s*\{\s*width\s*=\s*(\d+)", text, re.DOTALL)
        if not m:
            print("Could not parse sci cam width from: " + config_file)
            sys.exit(1)
        config_width = m.group(1)

    return float(image_width) / float(config_width)


def texture_mesh(src_path, cam_type, crop_win_map, mesh, args):
    if args.simulated_data and cam_type == "nav_cam":
        print("Texturing nav_cam is not supported with simulated data.")
        return "None"

    dist_image_list = os.path.join(args.output_dir, cam_type + "_index.txt")
    with open(dist_image_list) as f:
        image_files = f.readlines()
        if len(image_files) == 0:
            # That there are no images for a given camera is not necessarily fatal,
            # but do tell the user.
            print("Found no images for: " + cam_type)
            return ""

    dist_dir = os.path.join(args.output_dir, "distorted_" + cam_type)
    undist_dir = os.path.join(args.output_dir, "undistorted_" + cam_type)

    if not args.simulated_data:

        if cam_type == "sci_cam":
            # The sci cam needs special treatment
            scale = find_sci_cam_scale(image_files[0].rstrip())
        else:
            scale = 1.0

        # Make the crop win even, it is just easier that way
        undist_crop_win = [
            2 * int(round(float(crop_win_map[cam_type][0]) * scale / 2.0)),
            2 * int(round(float(crop_win_map[cam_type][1]) * scale / 2.0)),
        ]

        undistort_images(
            args, cam_type, dist_image_list, undist_dir, undist_crop_win, scale
        )
        create_texrecon_cameras(args, src_path, undist_dir, cam_type)
        textured_mesh = run_texrecon(args, src_path, mesh, undist_dir, cam_type)
    else:
        # Simulated images don't have distortion
        create_texrecon_cameras(args, src_path, dist_dir, cam_type)
        textured_mesh = run_texrecon(args, src_path, mesh, dist_dir, cam_type)

    return textured_mesh


def copy_dir(src, dst, symlinks=False, ignore=None):
    for item in os.listdir(src):
        s = os.path.join(src, item)
        d = os.path.join(dst, item)
        if os.path.isdir(s):
            shutil.copytree(s, d, symlinks, ignore)
        else:
            shutil.copy2(s, d)


def merge_poses_and_clouds(args):
    """
    Merge individual reconstructions from previous invocations of this tool.
    """
    input_dirs = args.merge_maps.split()

    if len(input_dirs) == 0:
        raise Exception("No input maps to merge were specified")

    # The out dir was created by now
    output_dir = args.output_dir

    # Copy all the data from the input dirs except log files
    # and sub-subdirectories
    print("Copying the data, this will take time.")
    for input_dir in input_dirs:
        items = os.listdir(input_dir)
        for item in items:

            if item.endswith("_log.txt"):
                continue

            # This copy is awkward because shutil.copytree cannot copy into an existing directory
            s = os.path.join(input_dir, item)
            d = os.path.join(output_dir, item)
            if not os.path.isdir(s):
                # is a file
                shutil.copy2(s, d)
            else:
                # is a directory
                if not os.path.isdir(d):
                    # Destination directory does not exist
                    shutil.copytree(s, d)
                else:
                    # Destination directory exists
                    items2 = os.listdir(s)
                    for item2 in items2:
                        # No further subirs are expected to be copied
                        s2 = os.path.join(s, item2)
                        d2 = os.path.join(d, item2)
                        if not os.path.isdir(s2):
                            shutil.copy2(s2, d2)

    # merge the index files
    index_files = os.listdir(output_dir)
    for index_file in index_files:
        if not index_file.endswith("index.txt"):
            continue

        # Merge the index files by concatenating them
        output_index = os.path.join(output_dir, index_file)
        f_out = open(output_index, "w")
        for input_dir in input_dirs:
            input_index = os.path.join(input_dir, index_file)
            if not os.path.isfile(input_index):
                continue
            f_in = open(input_index, "r")
            for line in f_in.readlines():
                line = line.replace(input_dir, output_dir)
                f_out.write(line)


if __name__ == "__main__":

    (src_path, exec_path, crop_win_map, args) = process_args(sys.argv)

    geometry_mapper_path = os.path.join(exec_path, "geometry_mapper")
    batch_tsdf_path = os.path.join(
        os.environ["HOME"], "catkin_ws/devel/lib/voxblox_ros/batch_tsdf"
    )

    sanity_checks(geometry_mapper_path, batch_tsdf_path, crop_win_map, args)

    setup_outputs(args)

    start_step = int(args.start_step)

    if start_step <= 0:
        if args.merge_maps == "":
            compute_poses_and_clouds(geometry_mapper_path, args)
        else:
            merge_poses_and_clouds(args)

    fused_mesh = os.path.join(args.output_dir, "fused_mesh.ply")
    if start_step <= 1 and args.external_mesh == "":
        fuse_clouds(batch_tsdf_path, fused_mesh, args)

    # Smothing must happen before hole-filling, to remove weird
    # artifacts
    smooth_mesh = os.path.join(args.output_dir, "smooth_mesh.ply")
    if start_step <= 2 and args.external_mesh == "" and (not args.simulated_data):
        attempt = 1
        smoothe_mesh(fused_mesh, smooth_mesh, args, attempt)

    # Fill holes
    hole_filled_mesh = os.path.join(args.output_dir, "hole_filled_mesh.ply")
    if start_step <= 3 and args.external_mesh == "" and (not args.simulated_data):
        attempt = 1
        fill_holes_in_mesh(smooth_mesh, hole_filled_mesh, args, attempt)

    # Rm small connected components
    clean_mesh = os.path.join(args.output_dir, "clean_mesh.ply")
    if start_step <= 4 and args.external_mesh == "" and (not args.simulated_data):
        rm_connected_components(hole_filled_mesh, clean_mesh, args)

    # Smoothe again
    smooth_mesh2 = os.path.join(args.output_dir, "smooth_mesh2.ply")
    if start_step <= 5 and args.external_mesh == "" and (not args.simulated_data):
        attempt = 2
        smoothe_mesh(clean_mesh, smooth_mesh2, args, attempt)

    # Fill holes again. That is necessary, and should happen after
    # smoothing.  Mesh cleaning creates small holes and they can't be
    # cleaned well without some more smoothing like above.
    hole_filled_mesh2 = os.path.join(args.output_dir, "hole_filled_mesh2.ply")
    if start_step <= 6 and args.external_mesh == "" and (not args.simulated_data):
        attempt = 2
        fill_holes_in_mesh(smooth_mesh2, hole_filled_mesh2, args, attempt)

    # Smoothe again as filling holes can make the mesh a little rough
    smooth_mesh3 = os.path.join(args.output_dir, "smooth_mesh3.ply")
    if start_step <= 7 and args.external_mesh == "" and (not args.simulated_data):
        attempt = 3
        smoothe_mesh(hole_filled_mesh2, smooth_mesh3, args, attempt)

    # Simplify the mesh
    simplified_mesh = os.path.join(args.output_dir, "simplified_mesh.ply")
    if start_step <= 8 and args.external_mesh == "" and (not args.simulated_data):
        simplify_mesh(smooth_mesh3, simplified_mesh, args)

    if args.simulated_data:
        simplified_mesh = fused_mesh
    elif args.external_mesh != "":
        simplified_mesh = args.external_mesh

    textured_meshes = []
    if start_step <= 9:
        for camera_type in args.camera_types.split():
            textured_mesh = texture_mesh(
                src_path, camera_type, crop_win_map, simplified_mesh, args
            )
            textured_meshes += [textured_mesh]

    if args.simulated_data:
        print("Fused mesh:                           " + fused_mesh)
    elif args.external_mesh == "":
        print("Fused mesh:                           " + fused_mesh)
        print("Smoothed mesh:                        " + smooth_mesh)
        print("Hole-filled mesh:                     " + hole_filled_mesh)
        print("Mesh with small componenents removed: " + clean_mesh)
        print("Further smoothed mesh:                " + smooth_mesh2)
        print("Further hole-filled mesh:             " + hole_filled_mesh2)
        print("Further smoothed mesh:                " + smooth_mesh3)
        print("Simplified mesh:                      " + simplified_mesh)
    else:
        print("External mesh: " + args.external_mesh)

    count = 0
    for camera_type in args.camera_types.split():
        print(camera_type + " textured mesh: " + textured_meshes[count])
        count += 1
