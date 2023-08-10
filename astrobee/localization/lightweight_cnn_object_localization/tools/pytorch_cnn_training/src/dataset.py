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
import json

# Third party imports
import numpy as np
import torch
import PIL

# Local imports
import transforms


class KeypointsDataset(torch.utils.data.Dataset):

    def __init__(self, dataset_path, is_train, bbox_size, images_dir="images", keypoints_file="keypoints.json"):
        """
        dataset_path: full path to dataset root
        is_train: if True, apply random augmentation
        bbox_size: half of the side length of the bboxes to generate
        images_dir: folder name for images folder, must be lcoated in dataset_path
        keypoints_file: json file containing keypoints, must be located in dataset_path
        """

        self.dataset_path = dataset_path
        self.images_dir = images_dir

        self.is_train = is_train
        transforms_to_apply = []
        if is_train:
            transforms_to_apply.append(transforms.RandAugment())
        transforms_to_apply.append(transforms.ToTensor())
        self.transforms_to_apply = transforms.Compose(transforms_to_apply)

        self.bbox_size = round(bbox_size)

        # find all image filepaths
        self.images_files = list(sorted(os.listdir(os.path.join(dataset_path, images_dir))))

        # load json file of keypoints and correspond them to images
        with open(os.path.join(dataset_path, keypoints_file), "r") as file:
            self.keypoints = json.load(file)
        

    def __getitem__(self, idx):

        # load image and configure to be RGB
        image_path = os.path.join(self.dataset_path, self.images_dir, self.images_files[idx])
        image = PIL.Image.open(image_path).convert("RGB")

        # get keypoints
        image_keypoints = self.keypoints[self.images_files[idx]]

        # unpack keypoints
        image_keypoints_bbox = []
        image_keypoints_class = []
        for image_keypoint in image_keypoints:
            x, y = image_keypoint["position"]
            ymax = y + self.bbox_size
            ymin = y - self.bbox_size
            xmax = x + self.bbox_size
            xmin = x - self.bbox_size
            if (0 <= xmin) and (xmax <= image.width) and (0 <= ymin) and (ymax <= image.height):
                image_keypoints_bbox.append([xmin, ymin, xmax, ymax])
                image_keypoints_class.append(image_keypoint["class"])

        # convert everything into a torch.Tensor
        if len(image_keypoints_bbox) > 0:
            image_keypoints_bbox = torch.as_tensor(image_keypoints_bbox, dtype=torch.float32)
            image_keypoints_class = torch.as_tensor(image_keypoints_class, dtype=torch.int64)
        else:
            image_keypoints_bbox = torch.zeros((0, 4), dtype=torch.float32)
            image_keypoints_class = torch.zeros(0, dtype=torch.int64)

        # construct target, apply transforms, and return
        target = {}
        target["boxes"] = image_keypoints_bbox
        target["labels"] = image_keypoints_class
        image, target = self.transforms_to_apply(image, target)
        return image, target

        # image_id = torch.tensor([idx])
        # area = (boxes[:, 3] - boxes[:, 1]) * (boxes[:, 2] - boxes[:, 0])
        # # suppose all instances are not crowd
        # iscrowd = torch.zeros((num_objs,), dtype=torch.int64)

        # target["masks"] = masks
        # target["image_id"] = image_id
        # target["area"] = area
        # target["iscrowd"] = iscrowd


    def __len__(self):
        return len(self.images_files)
