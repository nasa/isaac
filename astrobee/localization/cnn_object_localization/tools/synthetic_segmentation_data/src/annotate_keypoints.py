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
import json

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


def read_camera_parameters(world_file_path):
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
            horizontal_fov = float(horizontal_fov.text)  # radians assumed
            width = int(width.text)
            height = int(height.text)
            return horizontal_fov, width, height
    raise RuntimeError("Error parsing world file: segmentation_camera not found.")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tool_dir", type=str, required=True)
    parser.add_argument("--config_name", type=str, required=True)
    parser.add_argument("--data_dir", type=str, required=True)
    return parser.parse_args()


def main(tool_dir, config_name, data_dir):

    # Reading parameters
    config_dict = read_config(os.path.join(tool_dir, "config", f"{config_name}.config"))
    keypoints_target = json.loads(config_dict["KEYPOINT_POSITIONS"])
    assert len(set(len(pts) for pts in keypoints_target.values())) == 1  # all classes must have the same number of keypoints
    horizontal_fov, width, height = read_camera_parameters(os.path.join(tool_dir, "worlds", "templates", config_dict["WORLD_FILENAME"]))
    c_x = width / 2
    c_y = height / 2
    f = c_x / np.tan(horizontal_fov / 2)
    np_intrinsics_matrix = np.array([[f, 0, c_x], [0, f, c_y], [0, 0, 1]])

    # Load data
    df_ground_truth_poses = pd.read_csv(data_dir + "/groundTruthPoses.csv")
    df_inspection_poses = pd.read_csv(os.path.join(tool_dir, "inspection_poses", config_dict["INSPECTION_POSES_FILENAME"]))
    df_inspection_poses = df_inspection_poses[
        [config_dict[k] 
         for k 
         in ("INSPECTION_POSES_LABEL_NAME", *INSPECTION_POSES_TARGET_POSE_LABELS, "INSPECTION_POSES_LABEL_OBJECT_CLASS")]]
    df_merged = pd.merge(df_ground_truth_poses, df_inspection_poses, how="left", on="name")
    df_merged = df_merged.reset_index()

    # Extract inspection and target poses
    np_image_ids = df_merged.index.to_numpy().flatten()
    np_object_classes = df_merged[[config_dict["INSPECTION_POSES_LABEL_OBJECT_CLASS"]]].to_numpy().flatten().astype(str)
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

    # For each image, generate a mask for keypoints
    # os.mkdir(os.path.join(data_dir, "keypoint_annotated"))  # (for debugging purposes only)
    os.mkdir(os.path.join(data_dir, "keypoint_masks"))
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

        # Load, annotate and save image (for debugging purposes only)
        # image_path_original = os.path.join(data_dir, "images", f"image_{(np_image_ids[image_idx]):07d}.png")
        # image_annotated = cv2.imread(image_path_original)
        # for np_keypoint in keypoints_image:
        #     image_annotated = cv2.circle(image_annotated, np_keypoint.flatten().astype(int), int(config_dict["KEYPOINT_RADIUS"]), (0, 0, 255), -1)
        # image_path_annotated = os.path.join(data_dir, "keypoint_annotated", f"annotated_{(np_image_ids[image_idx]):07d}.png")
        # cv2.imwrite(image_path_annotated, image_annotated)

        # Generate a mask for keypoints
        image_keypoint_mask = np.zeros((height,width,3), np.uint8)
        keypoint_color = 80  # completely arbitrary
        for np_keypoint in keypoints_image:
            image_keypoint_mask = cv2.circle(image_keypoint_mask, np_keypoint.flatten().astype(int), int(config_dict["KEYPOINT_RADIUS"]), (keypoint_color, 0, 0), -1)
            keypoint_color += 10  # also completely arbitrary
        image_path_keypoint_mask = os.path.join(data_dir, "keypoint_masks", f"colored_{(np_image_ids[image_idx]):07d}.png")
        cv2.imwrite(image_path_keypoint_mask, image_keypoint_mask)


if __name__ == "__main__":

    args = parse_args()
    main(
        tool_dir=args.tool_dir,
        config_name=args.config_name,
        data_dir=args.data_dir)
    





    







