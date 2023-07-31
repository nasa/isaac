#!/usr/bin/python
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
import os

# Third party imports
import cv2
import numpy as np
from PIL import Image


# Add the colored map to the image for visualization
def add_colored_to_image(image, colored):
    return cv2.addWeighted(
        cv2.resize(image, (colored.shape[1], colored.shape[0])).astype(np.uint8),
        1,
        colored.astype(np.uint8),
        0.5,
        0,
        cv2.CV_32F,
    )


coloring_scheme = {1: [55, 255, 20], 2: [0, 200, 172], 3: [0, 106, 200], 4: [0, 173, 2]}


def convert_mask_to_image(mask, label):
    colored_map = np.array(Image.fromarray(mask).convert("RGB"))
    colored_map[np.where(mask == label)] = coloring_scheme[label]
    return colored_map


def visualize(image, bbox, mask, label):
    # image = cv2.imread(img_path, cv2.IMREAD_COLOR)
    colored_map = convert_mask_to_image(mask, label)
    colored_image = add_colored_to_image(image, colored_map)
    return cv2.rectangle(
        colored_image,
        (int(np.floor(bbox[0])), int(np.floor(bbox[1]))),
        (int(np.ceil(bbox[2])), int(np.ceil(bbox[3]))),
        coloring_scheme[label],
        1,
    )


def save_image(annotated_img, img_path):
    print(annotated_img)
    path_save = "data_test/output/"
    imageId = "segmentation_" + os.path.splitext(img_path)[0][-7:] + ".png"
    # if not cv2.imwrite(path_save + imageId, annotated_img):
    #     raise Exception("Could not write image")
    cv2.imshow("Output", annotated_img)

    # waits for user to press any key
    # (this is necessary to avoid Python kernel form crashing)
    cv2.waitKey(0)

    # closing all open windows
    cv2.destroyAllWindows()
