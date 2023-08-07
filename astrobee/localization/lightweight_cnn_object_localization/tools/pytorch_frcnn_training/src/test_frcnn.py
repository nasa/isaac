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
import os

# Third party imports
import cv2
import numpy as np
import torch
from PIL import Image
from torchvision import transforms
from tqdm import tqdm

# Local imports
from model import get_model


def post_process(detections, num_detections, c_thresh=0.75):
    p_detections = []
    for i in range(num_detections):
        if detections["scores"][i] > c_thresh:
            detection = {
                "bbox": detections["boxes"]
                .detach()
                .numpy()[i]
                .reshape(
                    4,
                ),
                "label": detections["labels"].detach().numpy()[i],
            }
            p_detections.append(detection)
    return p_detections


def main(dataset_path: str, output_path: str, weights_path: str):

    torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    # our dataset has two classes (background, plus keypoint; all keypoints are the same right now)
    num_classes = 2

    # get the trained model
    model = get_model(num_classes, weights_path=weights_path)
    model.eval()

    # inference on images
    convert_tensor = transforms.ToTensor()
    img_paths = [
        os.path.join(dataset_path, img_name)
        for img_name 
        in list(sorted(os.listdir(dataset_path)))]
    img_out_paths = [
        os.path.join(output_path, img_name)
        for img_name 
        in list(sorted(os.listdir(dataset_path)))]
    os.mkdir(output_path)
    for img_path, img_out_path in tqdm(zip(img_paths, img_out_paths)):

        # load image
        img = Image.open(img_path).convert("RGB")
        img = [convert_tensor(img)]
        if torch.cuda.is_available():
            torch.cuda.synchronize()

        # run model
        detections = model(img)[0]
        n = len(detections["labels"])
        detections = post_process(detections, n)

        # annotate image with output
        annotated_img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        for detection in detections:
            bbox, label = detection.values()
            annotated_img = cv2.rectangle(
                annotated_img,
                (int(np.floor(bbox[0])), int(np.floor(bbox[1]))),
                (int(np.ceil(bbox[2])), int(np.ceil(bbox[3]))),
                [55, 255, 20],
                1)
        cv2.imwrite(img_out_path, annotated_img)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--dataset_path", type=str, required=True)
    parser.add_argument("-o", "--output_path", type=str, required=True)
    parser.add_argument("-w", "--weights_path", type=str, required=True)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    main(
        dataset_path=args.dataset_path,
        output_path=args.output_path,
        weights_path=args.weights_path,
    )

