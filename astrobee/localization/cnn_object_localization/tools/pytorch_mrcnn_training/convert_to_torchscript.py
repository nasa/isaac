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
import numpy as np
import torch
import torchvision
from PIL import Image
from torchvision import transforms
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor


convert_tensor = transforms.ToTensor()


def post_process(detections, num_detections, c_thresh=0.75):
    # print(detections)
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
                "mask": detections["masks"].detach().numpy()[i].reshape(240, 320),
                "label": detections["labels"].detach().numpy()[i],
            }
            p_detections.append(detection)
    return p_detections


def get_trained_model(weights_path, num_classes=5):
    # load an instance segmentation model pre-trained on COCO
    model = torchvision.models.detection.maskrcnn_resnet50_fpn()
    # replace the pre-trained head with a new one
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
    # and replace the mask predictor with a new one
    in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
    hidden_layer = 256
    model.roi_heads.mask_predictor = MaskRCNNPredictor(
        in_features_mask, hidden_layer, num_classes
    )
    # load weights
    model.load_state_dict(torch.load(weights_path))
    return model


def main(dataset_path: str, weights_path: str):

    model = get_trained_model(weights_path)
    model.eval()

    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    img_path = os.path.join(dataset_path, os.listdir(dataset_path)[0])
    img = Image.open(img_path).convert("RGB")
    img = [[convert_tensor(img)]]
    torch.cuda.synchronize()
    traced_model = torch.jit.script(model, img)

    output_dir, weights_name_ext = os.path.split(weights_path)
    weights_name, _ = os.path.splitext(weights_name_ext)
    output_name_ext = weights_name + "_torchscript.pt"
    output_path = os.path.join(output_dir, output_name_ext)
    traced_model.save(output_path)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--dataset_path", type=str, required=True)
    parser.add_argument("-w", "--weights_path", type=str, default=None)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    main(
        dataset_path=args.dataset_path,
        weights_path=args.weights_path,
    )

