#!/usr/bin/env python3
#
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


# Python imports
import argparse
from xml.etree import ElementTree
import os

# Third party imports
import numpy as np
import pandas as pd
import cv2

# Local imports
from utils import transform


INSPECTION_POSES_TARGET_POSE_LABELS = (
    "INSPECTION_POSES_LABEL_TARGET_POS_X", 
    "INSPECTION_POSES_LABEL_TARGET_POS_Y", 
    "INSPECTION_POSES_LABEL_TARGET_POS_Z", 
    "INSPECTION_POSES_LABEL_TARGET_ROT_X", 
    "INSPECTION_POSES_LABEL_TARGET_ROT_Y", 
    "INSPECTION_POSES_LABEL_TARGET_ROT_Z")
GROUND_TRUTH_POSE_LABELS = (
    "GROUND_TRUTH_LABEL_POS_X", 
    "GROUND_TRUTH_LABEL_POS_Y", 
    "GROUND_TRUTH_LABEL_POS_Z", 
    "GROUND_TRUTH_LABEL_ROT_X", 
    "GROUND_TRUTH_LABEL_ROT_Y", 
    "GROUND_TRUTH_LABEL_ROT_Z")


def read_config(config_file_path):
    config_dict = {}
    with open(config_file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line and not line.startswith('#'):
                key, value = line.split('=', 1)
                config_dict[key] = value
    return config_dict


def read_camera_intrinsics(world_file_path):
    tree = ElementTree.parse(world_file_path)
    root = tree.getroot()
    for sensor in root.iter('sensor'):
        if 'name' in sensor.attrib and sensor.attrib['name'] == 'segmentation_camera':
            camera = sensor.find('camera')
            if camera is None:
                raise RuntimeError("Error parsing world file: <camera> not found.")
            horizontal_fov = camera.find('horizontal_fov')
            if horizontal_fov is None:
                raise RuntimeError("Error parsing world file: <horizontal_fov> not found.")
            image = camera.find('image')
            if image is None:
                raise RuntimeError("Error parsing world file: <image> not found.")
            width = image.find('width')
            height = image.find('height')
            if (width is None) or (height is None):
                raise RuntimeError("Error parsing world file: <width> and/or <height> not found.")
            fov_x = float(horizontal_fov.text)  # radians assumed
            c_x = float(width.text) / 2
            c_y = float(height.text) / 2
            f = c_x / np.tan(fov_x / 2)
            return np.array([[f, 0, c_x], [0, f, c_y], [0, 0, 1]])
    raise RuntimeError("Error parsing world file: segmentation_camera not found.")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config_name", type=str, required=True)
    return parser.parse_args()


def main(config_name):

    # Directories
    # tool_dir = "/usr/local/home/mnsun/ros_ws/isaac/src/astrobee/localization/cnn_object_localization/tools/synthetic_segmentation_data"
    # data_dir = "/usr/local/home/mnsun/large_files/data/handrail/synthetic/new_with_gt"
    tool_dir = "/home/astrobee/ros_ws/isaac/src/astrobee/localization/cnn_object_localization/tools/synthetic_segmentation_data"
    data_dir = "/home/astrobee/large_files/data/handrail/synthetic/new_with_gt"

    # Reading parameters and data
    config_dict = read_config(os.path.join(tool_dir, "config", f"{config_name}.config"))
    np_intrinsics_matrix = read_camera_intrinsics(os.path.join(tool_dir, "worlds", "templates", config_dict["WORLD_FILENAME"]))

    # Keypoints in the target frame
    # In the future, this should be read from config as well
    keypoints_target = {
        100: [[0.06, 0, 0.09], [0.06, 0, -0.09]],
        120: [[0.06, 0, 0.25], [0.06, 0, -0.25]],
        140: [[0.06, 0, 0.36], [0.06, 0, -0.36]],
        180: [[0.06, 0, 0.51], [0.06, 0, -0.51]],
    }

    # Load data
    df_ground_truth_poses = pd.read_csv(data_dir + "/groundTruthPoses.csv")
    df_inspection_poses = pd.read_csv(os.path.join(tool_dir, "inspection_poses", config_dict["INSPECTION_POSES_FILENAME"]))
    df_inspection_poses = df_inspection_poses[
        [config_dict[k] 
         for k 
         in ("INSPECTION_POSES_LABEL_NAME", *INSPECTION_POSES_TARGET_POSE_LABELS, "INSPECTION_POSES_LABEL_OBJECT_CLASS")]]
    df_merged = pd.merge(df_ground_truth_poses, df_inspection_poses, how="left", on="name")
    df_merged.index = np.arange(1, len(df_merged) + 1)  # reset index starting at 1, not 0

    # Extract inspection and target poses
    np_image_ids = df_merged.index.to_numpy().flatten()
    np_object_classes = df_merged[[config_dict["INSPECTION_POSES_LABEL_OBJECT_CLASS"]]].to_numpy().flatten()
    np_inspection_poses = df_merged[  # randomized camera poses
        [config_dict[key] 
         for key 
         in GROUND_TRUTH_POSE_LABELS]].to_numpy()
    np_inspection_positions = np_inspection_poses[:, 0:3]
    np_inspection_eulangles = np_inspection_poses[:, 3:6]
    np_target_poses = df_merged[  # static handrail poses
        [config_dict[key] 
         for key 
         in INSPECTION_POSES_TARGET_POSE_LABELS]].to_numpy()
    np_target_positions = np_target_poses[:, 0:3]
    np_target_eulangles = np_target_poses[:, 3:6]

    # For each image, transform keypoints into image coordinates
    for image_idx in range(len(df_merged)):

        # Get transformation matrix
        np_transformation_matrix = transform.transform_between_frames(
            np_inspection_positions[image_idx], np_inspection_eulangles[image_idx],
            np_target_positions[image_idx], np_target_eulangles[image_idx])
        np_transformation_matrix_correction = np.array(  # necessary because of the way camera frame is defined in gazebo
            [[ 0, 1, 0, 0],
             [ 0, 0, 1, 0],
             [-1, 0, 0, 0],
             [ 0, 0, 0, 1]])
        np_transformation_matrix = np.matmul(np_transformation_matrix_correction, np_transformation_matrix)
        
        # Transform keypoints to camera frame
        keypoints_camera = [
            transform.apply_transformation(np_transformation_matrix, p) 
            for p 
            in keypoints_target[np_object_classes[image_idx]]]

        # Transform keypoints to image frame
        keypoints_homogeneous_image = [np.dot(np_intrinsics_matrix, p) for p in keypoints_camera]
        keypoints_image = [p[:2] / p[2] for p in keypoints_homogeneous_image]

        # Load, annotate and save image
        image_name_original = f"image_{(np_image_ids[image_idx]):07d}.png"
        image_name_annotated = f"annotated_{(np_image_ids[image_idx]):07d}.png"
        image_path = os.path.join(data_dir, "images", image_name_original)
        image = cv2.imread(image_path)
        for np_keypoint in keypoints_image:
            image = cv2.circle(image, np_keypoint.flatten().astype(int), 10, (0, 0, 255), -1)
        cv2.imwrite(image_name_annotated, image)






    



if __name__ == "__main__":

    args = parse_args()
    main(
        config_name=args.config_name)
    





    







