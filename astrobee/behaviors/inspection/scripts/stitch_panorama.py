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
Generate/update Hugin PTO files and stitch a panorama.

Example:
  export ASTROBEE_RESOURCE_DIR=$SOURCE_PATH/astrobee/resources
  export ASTROBEE_CONFIG_DIR=$SOURCE_PATH/astrobee/config
  export ASTROBEE_WORLD=granite
  export ASTROBEE_ROBOT=bsharp
  rosrun inspection scripts/stitch_panorama.py in.bag --images-dir=isaac_sci_cam_delayed
"""

import argparse
import datetime
import math
import os
import shutil
import subprocess
import sys

import cv2
import hsi
import rosbag
from tf.transformations import euler_from_quaternion

RAD2DEG = 180 / math.pi
IMAGE_TOPIC = "/hw/cam_sci/compressed"
POSE_TOPIC = "/loc/pose"
UNDISTORT_ENV_VARS = [
    "ASTROBEE_RESOURCE_DIR",
    "ASTROBEE_CONFIG_DIR",
    "ASTROBEE_WORLD",
    "ASTROBEE_ROBOT",
]
# List of Hugin variables that relate to camera intrinsics. There are many
# more, but these are the only ones we optimize.
LENS_PARAMS = ("v", "b")
TRANSLATION_PARAMS = ("TrX", "TrY", "TrZ", "Tpy", "Tpp")


def quote_if_needed(arg):
    if " " in arg:
        return '"' + arg + '"'
    else:
        return arg


def run_cmd(cmd, path_prefix=None):
    print("run_cmd: " + " ".join([quote_if_needed(arg) for arg in cmd]))

    if path_prefix:
        env = os.environ.copy()
        env["PATH"] = path_prefix + ":" + env["PATH"]
        popen = subprocess.Popen(cmd, env=env)
    else:
        popen = subprocess.Popen(cmd)
    return_code = popen.wait()
    if return_code:
        raise subprocess.CalledProcessError(return_code, cmd)

    return return_code


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    pass


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "inbag",
        type=str,
        help="Input bagfile containing SciCam images and pose messages.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        required=False,
        default="stitch_{inbag}",
        help="Directory to write stitched pano and intermediate files",
    )
    parser.add_argument(
        "--images-dir",
        type=str,
        required=False,
        default=".",
        help="Directory to read SciCam images from.",
    )
    parser.add_argument(
        "--no-stitching",
        dest="no_stitching",
        action="store_true",
        default=False,
        help="Generate hugin and optimize only.",
    )
    parser.add_argument(
        "--stitching-only",
        dest="only_stitching",
        action="store_true",
        default=False,
        help="Stitch hugin file only",
    )
    parser.add_argument(
        "--no-undistort",
        action="store_true",
        default=False,
        help="Don't undistort input images before stitching",
    )
    parser.add_argument(
        "--no-lens",
        action="store_true",
        default=False,
        help="Disable optimization of lens parameters",
    )
    parser.add_argument(
        "--no-translation",
        action="store_true",
        default=False,
        help="Disable optimization of translation parameters",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        default=False,
        help="Disable duplicating console output to log file",
    )
    parser.add_argument(
        "--skip-images",
        type=str,
        required=False,
        default=None,
        help="Skip specified images. Comma-separated list of substrings to match image file against (you can just specify the timestamps of the images you want to skip).",
    )

    args = parser.parse_args()

    return args


def get_image_meta(inbag_path, images_dir, skip_images_str):
    src_images = []
    if skip_images_str is None:
        skip_images = []
    else:
        skip_images = skip_images_str.split(",")
    with rosbag.Bag(inbag_path) as bag:
        need_pose = False
        print("Detecting images:")
        for topic, msg, t in bag.read_messages([IMAGE_TOPIC, POSE_TOPIC]):
            if topic == IMAGE_TOPIC:
                img_path = os.path.join(
                    images_dir,
                    str(msg.header.stamp.secs)
                    + "."
                    + "%03d" % (msg.header.stamp.nsecs * 0.000001)
                    + ".jpg",
                )
                print("  " + img_path)

                img_base = os.path.basename(img_path)
                skip_matches = [s for s in skip_images if s in img_base]
                if skip_matches:
                    print(
                        "    Matches one of --skip-images (%s), skipping"
                        % skip_matches[0]
                    )
                    continue

                # Make sure image exists
                if not os.path.exists(img_path):
                    raise RuntimeError("Could not find %s" % img_path)

                # Gather image metadata
                src_image = hsi.SrcPanoImage()
                src_image.setVar("v", 62)  # about right, may override later
                src_image.setFilename(os.path.realpath(img_path))
                src_images.append(src_image)

                need_pose = True

            if need_pose and topic == POSE_TOPIC:
                # Configure hugin parameters
                orientation_list = [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                src_image.setVar("r", roll * RAD2DEG)
                src_image.setVar("p", pitch * RAD2DEG)
                src_image.setVar("y", yaw * RAD2DEG)

                # If y/p are very far from Tpy/Tpp (difference approaching 90
                # degrees or greater), the remapping math gets weird and nona
                # can produce nonsensical output.  And not sure how to
                # interpret Tpy/Tpp. Like either or both could have opposite
                # sign from y/p. It's clear there's a real problem here, but
                # this change to try and help if anything made it worse so far.
                # https://hugin.sourceforge.io/docs/manual/Stitching_a_photo-mosaic.html
                # https://wiki.panotools.org/Hugin_FAQ - search "translation"
                # src_image.setVar("Tpy", yaw * RAD2DEG)
                # src_image.setVar("Tpp", pitch * RAD2DEG)

                need_pose = False

    return src_images


def get_undistorted_path(fname, undistort_dir, ext):
    with_ext = os.path.splitext(os.path.basename(fname))[0] + ext
    return os.path.realpath(os.path.join(undistort_dir, with_ext))


def undistort_images(src_images, output_dir):
    undistort_dir = os.path.join(output_dir, "build", "undistort")
    intrinsics_path = os.path.join(undistort_dir, "undistorted_intrinsics.txt")

    call_undistort(src_images, undistort_dir, intrinsics_path)

    # point pto file at the undistorted images
    for img in src_images:
        img.setFilename(get_undistorted_path(img.getFilename(), undistort_dir, ".png"))

    undistorted_intrinsics = read_undistorted_intrinsics(intrinsics_path)
    set_h_fov(src_images, undistorted_intrinsics)


def call_undistort(src_images, undistort_dir, intrinsics_path):
    if os.path.exists(intrinsics_path):
        print(
            "Undistort output %s already exists, not reprocessing images"
            % intrinsics_path
        )
        return

    # create file listing input images for undistort_image
    if not os.path.exists(undistort_dir):
        os.makedirs(undistort_dir)

    input_images_path = os.path.join(undistort_dir, "input_images.txt")
    with open(input_images_path, "w") as input_list:
        for img in src_images:
            input_list.write(img.getFilename() + "\n")

    # check necessary environment variables are defined
    for var in UNDISTORT_ENV_VARS:
        if os.getenv(var) is None:
            raise RuntimeError(
                "Environment variable %s must be defined in order for undistort_image to read the correct camera parameters"
                % var
            )

    run_cmd(
        [
            "rosrun",
            "camera",
            "undistort_image",
            "-image_list",
            input_images_path,
            "-robot_camera",
            "sci_cam",
            # default output size is much larger than needed, enable autocrop
            "-undistorted_crop_win",
            "loose",
            "-alpha",
            "-cubic",
            "-output_directory",
            undistort_dir,
            "-undistorted_intrinsics",
            intrinsics_path,
        ]
    )


def read_undistorted_intrinsics(intrinsics_path):
    with open(intrinsics_path, "r") as intrinsics_stream:
        lines = intrinsics_stream.read().splitlines()
        vals = lines[1].split()
        # all values have units of pixels
        intrinsics = {
            "width": float(vals[0]),
            "height": float(vals[1]),
            "focal_length": float(vals[2]),
            "center_x": float(vals[3]),
            "center_y": float(vals[4]),
        }
    return intrinsics


def get_h_fov(undistorted_intrinsics):
    # see https://wiki.panotools.org/Field_of_View "Conversion from focal length"
    width = undistorted_intrinsics["width"]
    f = undistorted_intrinsics["focal_length"]
    h_fov_degrees = RAD2DEG * 2 * math.atan(width / (2 * f))
    return h_fov_degrees


def set_h_fov(src_images, undistorted_intrinsics):
    h_fov_degrees = get_h_fov(undistorted_intrinsics)
    for img in src_images:
        img.setVar("v", h_fov_degrees)


def concat_if(prefix, suffix, cond):
    if cond:
        return prefix + suffix
    else:
        return prefix


def filter_params1(params, reject_params, do_filter):
    if do_filter:
        return ",".join([p for p in params.split(",") if p not in reject_params])
    else:
        return params


def filter_params(params, args):
    params = filter_params1(params, LENS_PARAMS, args.no_lens)
    params = filter_params1(params, TRANSLATION_PARAMS, args.no_translation)
    return params


class PathSequence(object):
    """
    Generates a sequence of filenames based on a "base" filename
    by inserting a number "_NNN" before the extension.
    """

    def __init__(self, base_path, insert_n):
        self.base_path = base_path
        self.insert_n = insert_n
        self.n = 0

    def insert_suffix(self, suffix):
        prefix, ext = os.path.splitext(self.base_path)
        return prefix + suffix + ext

    def get_path(self):
        if self.insert_n:
            return self.insert_suffix("_%03d" % self.n)
        else:
            return self.base_path

    def next(self):
        self.n += 1
        return self.get_path()


def read_pto(pano, pto_path):
    print("\nread_pto: %s" % pto_path)
    ifs = hsi.ifstream(pto_path)
    pano.readData(ifs)


def write_pto(pano, pto_path):
    print("\nwrite_pto: %s" % pto_path)
    ofs = hsi.ofstream(pto_path)
    pano.writeData(ofs)


def get_timestamp():
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def duplicate_console_to_log(log_path):
    """
    Duplicate console output to specified log file. Both stdout
    and stderr of this process and all children should be included.
    """
    print("Duplicating console log to %s" % log_path)
    log_path = os.path.realpath(log_path)
    log_dir = os.path.dirname(log_path)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # Unbuffer stdout (ensures stdout and stderr interleave properly). As of
    # Python 3.3+ this might not work and no longer be needed.
    sys.stdout = os.fdopen(sys.stdout.fileno(), "w", 0)

    tee = subprocess.Popen(["tee", "--append", log_path], stdin=subprocess.PIPE)
    os.dup2(tee.stdin.fileno(), sys.stdout.fileno())
    os.dup2(tee.stdin.fileno(), sys.stderr.fileno())


def main():
    args = parse_args()

    inbag_base = os.path.splitext(os.path.basename(args.inbag))[0]
    output_dir = args.output_dir.format(inbag=inbag_base)

    if not args.no_log:
        log_path = os.path.join(output_dir, "build", "console.log")
        duplicate_console_to_log(log_path)

    print("%s Started run" % get_timestamp())
    print("Command-line arguments: %s" % sys.argv)

    pto_base = os.path.join(output_dir, "build", "stitch.pto")
    pto = PathSequence(pto_base, True)
    pto_final = pto.insert_suffix("_final")

    if not args.only_stitching:
        src_images = get_image_meta(args.inbag, args.images_dir, args.skip_images)
        if not args.no_undistort:
            undistort_images(src_images, output_dir)

        # Make a new Panorama object
        p = hsi.Panorama()
        for img in src_images:
            p.addImage(img)

        # Link lenses
        variable_groups = hsi.StandardImageVariableGroups(p)
        lenses = variable_groups.getLenses()
        for i in range(0, p.getNrOfImages()):
            lenses.switchParts(i, lenses.getPartNumber(0))

        write_pto(p, pto.get_path())

        # Generate control points
        cmd = [
            "cpfind",
            "--multirow",
            # Request more control points 2 -> 5 in case it helps optimization.
            "--sieve2size",
            "5",
            # Per cpfind man page, rpy is "the preferred mode if a calibrated
            # lens is used"
            "--ransacmode",
            "rpy",
            pto.get_path(),
            "-o",
            pto.next(),
        ]
        run_cmd(cmd)

        # Throw away control points are prob invalid
        cmd = ["cpclean", pto.get_path(), "-o", pto.next()]
        run_cmd(cmd)

        # Optimize attitude + b
        cmd = [
            "pto_var",
            "--opt",
            filter_params("y,p,r,b", args),
            pto.get_path(),
            "-o",
            pto.next(),
        ]
        run_cmd(cmd)

        current = pto.get_path()
        cmd = ["autooptimiser", "-n", "-o", pto.next(), current]
        run_cmd(cmd)

        # Optimize position iteratively not to diverge
        read_pto(p, pto.get_path())

        orig_tuple = p.getOptimizeVector()

        pano_size = len(orig_tuple)
        stride = 5
        optim_nr_list = [10, 15]

        for optim_nr in optim_nr_list:

            # Optimizer cycles
            # We do this iteratively otherwise it will diverge
            for x in range(0, pano_size, stride):
                print(x)
                read_pto(p, pto.get_path())
                optvec = []
                for i in range(pano_size):
                    if (x + optim_nr) > pano_size:
                        start = x + optim_nr - pano_size
                    else:
                        start = 0
                    if x <= i < x + optim_nr or i < start:
                        optvec.append(
                            filter_params("y,p,r,Tpp,Tpy,TrX,TrY,TrZ", args).split(",")
                        )
                    else:
                        optvec.append([])

                print(optvec)
                p.setOptimizeVector(optvec)
                write_pto(p, pto.next())

                current = pto.get_path()
                cmd = ["autooptimiser", "-n", "-o", pto.next(), current]
                run_cmd(cmd)

            # Optimize attitude + b + v
            cmd = [
                "pto_var",
                "--opt",
                filter_params("y,p,r,b,v", args),
                pto.get_path(),
                "-o",
                pto.next(),
            ]
            run_cmd(cmd)

            current = pto.get_path()
            cmd = ["autooptimiser", "-n", "-o", pto.next(), current]
            run_cmd(cmd)

        # Optimize attitude + position
        cmd = [
            "pto_var",
            "--opt",
            filter_params("y,p,r,Tpp,Tpy,TrX,TrY,TrZ", args),
            pto.get_path(),
            "-o",
            pto.next(),
        ]
        run_cmd(cmd)

        current = pto.get_path()
        cmd = ["autooptimiser", "-n", "-o", pto.next(), current]
        run_cmd(cmd)

        # Optimize ALL x3
        cmd = [
            "pto_var",
            "--opt",
            filter_params("y,p,r,Tpp,Tpy,TrX,TrY,TrZ,b,v", args),
            pto.get_path(),
            "-o",
            pto.next(),
        ]
        run_cmd(cmd)

        current = pto.get_path()
        cmd = ["autooptimiser", "-n", "-o", pto.next(), current]
        run_cmd(cmd)

        cmd = [
            "pto_var",
            "--opt",
            filter_params("y,p,r,TrX,TrY,TrZ,b,v", args),
            pto.get_path(),
            "-o",
            pto.next(),
        ]
        run_cmd(cmd)

        current = pto.get_path()
        cmd = ["autooptimiser", "-n", "-o", pto.next(), current]
        run_cmd(cmd)

        cmd = [
            "pto_var",
            "--opt",
            filter_params("y,p,r,Tpp,Tpy,TrX,TrY,TrZ,b,v", args),
            pto.get_path(),
            "-o",
            pto.next(),
        ]
        run_cmd(cmd)

        current = pto.get_path()
        cmd = ["autooptimiser", "-n", "-o", pto.next(), current]
        run_cmd(cmd)

        # Photometric Optimizer
        current = pto.get_path()
        cmd = ["autooptimiser", "-m", "-o", pto.next(), current]
        run_cmd(cmd)

        # Configure image output
        cmd = [
            "pano_modify",
            "--center",
            "--canvas=AUTO",
            "--projection=2",
            "--fov=360x180",
            "--output-type=NORMAL,REMAP",
            # Use lossless TIFF 'deflate' compression in output
            # images. Empirically, this makes them smaller but not as small as
            # PNG, so we'll do one more conversion at the end.
            "--ldr-compression=DEFLATE",
            pto.get_path(),
            "-o",
            pto.next(),
        ]
        run_cmd(cmd)

        # Set enblend options for stitching
        read_pto(p, pto.get_path())

        # Black dropout areas workaround #1: Switch primary seam generator to
        # less advanced but possibly more robust older version. This is probably
        # the better way to handle it (increases speed as well). Some places
        # online recommend trying this.
        p.getOptions().enblendOptions += " --primary-seam-generator=nft"

        # Black dropout areas workaround #2: Turn off seam optimization. This
        # seems to help in some cases but not as often? May not be needed if
        # workaround #1 is used.
        # p.getOptions().enblendOptions += " --no-optimize --fine-mask"

        print("Set enblend options: %s" % p.getOptions().enblendOptions)
        write_pto(p, pto.next())

        # Need to copy last pto in sequence to be "final", so it will be used
        # on any subsequent --stitching-only runs.
        shutil.copyfile(pto.get_path(), pto_final)

    if not args.no_stitching:
        print("%s Starting stitching" % get_timestamp())
        # Generate panorama
        pano_path = os.path.join(output_dir, "build", "pano")
        cmd = [
            "hugin_executor",
            "--stitching",
            "--prefix=" + pano_path,
            pto_final,
        ]

        print("\n=== Stitching first in dry run mode for debugging ===")
        dry_run_cmd = list(cmd)
        dry_run_cmd[-1:-1] = ["--dry-run"]
        run_cmd(dry_run_cmd)

        print("\n=== Now stitching for real ===")
        run_cmd(cmd)

        tif_path = pano_path + ".tif"
        final_png_path = os.path.join(output_dir, os.path.basename(pano_path)) + ".png"
        print("opencv_convert %s %s" % (tif_path, final_png_path))
        pano_img = cv2.imread(tif_path)
        cv2.imwrite(final_png_path, pano_img)
        print("\n=== Final stitched pano in %s ===\n" % final_png_path)

    print("%s Finished run" % get_timestamp())
    if not args.no_log:
        print("Complete console output in %s" % log_path)


if __name__ == "__main__":
    main()
