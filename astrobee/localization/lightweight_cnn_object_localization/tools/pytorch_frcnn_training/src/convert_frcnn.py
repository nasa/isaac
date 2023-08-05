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
import torch
from PIL import Image
from torchvision import transforms

# Local imports
from model import get_model


def main(dataset_path: str, weights_path: str):

    torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    # our dataset has two classes (background, plus keypoint; all keypoints are the same right now)
    num_classes = 2

    # get the trained model
    model = get_model(num_classes, weights_path=weights_path)
    model.eval()

    # we need an image for dummy inference
    convert_tensor = transforms.ToTensor()
    img_path = os.path.join(dataset_path, os.listdir(dataset_path)[0])
    img = Image.open(img_path).convert("RGB")
    img = [convert_tensor(img)]
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    traced_model = torch.jit.script(model, img)

    # save the traced model
    output_dir, weights_name_ext = os.path.split(weights_path)
    weights_name, _ = os.path.splitext(weights_name_ext)
    output_name_ext = weights_name + "_torchscript.pt"
    output_path = os.path.join(output_dir, output_name_ext)
    traced_model.save(output_path)

    # load the saved model and make sure it works
    compiled_model = torch.jit.load(output_path)
    compiled_model.eval()
    img = Image.open(img_path).convert("RGB")
    img = [convert_tensor(img)]  
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    detections = model(img)[0]
    # print(detections)



def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--dataset_path", type=str, required=True)
    parser.add_argument("-w", "--weights_path", type=str, required=True)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    main(
        dataset_path=args.dataset_path,
        weights_path=args.weights_path,
    )

