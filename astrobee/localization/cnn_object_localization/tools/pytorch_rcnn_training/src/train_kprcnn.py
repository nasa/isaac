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
from typing import Optional
import sys
import math
import os

# Third party imports
import torch
import torchvision
import tqdm
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor

# Local imports
import utils.transforms as T
import utils.utils as utils
from utils.dataset import AstrobeeHandrailDataset
from utils.engine import evaluate
from utils.model_params import Model_Params


def get_transform(train):
    transforms = []
    if train:
        transforms.append(T.RandAugment())
    transforms.append(T.ToTensor())
    return T.Compose(transforms)


def get_model_instance_segmentation(num_classes):
    # load an instance segmentation model pre-trained on COCO
    model = torchvision.models.detection.maskrcnn_resnet50_fpn(pretrained=True)

    # get number of input features for the classifier
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    # replace the pre-trained head with a new one
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

    # now get the number of input features for the mask classifier
    in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
    hidden_layer = 256
    # and replace the mask predictor with a new one
    model.roi_heads.mask_predictor = MaskRCNNPredictor(
        in_features_mask, hidden_layer, num_classes
    )
    return model


def main(
        n_epochs: int, 
        dataset_path: str, 
        output_path: str, output_label: Optional[str] = None, 
        init_weights_path: Optional[str] = None):

    params = Model_Params()
    # train on the GPU or on the CPU, if a GPU is not available
    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    # our dataset has two classes only - background and keypoint
    num_classes = 3
    # use our dataset and defined transformations
    label_dict = {9: 1, 10: 1}  # two keypoint instances per image, both of the same class
    dataset = AstrobeeHandrailDataset(dataset_path, get_transform(train=True), label_dict=label_dict, images_dir="images", masks_dir="keypoint_masks")
    dataset_test = AstrobeeHandrailDataset(dataset_path, get_transform(train=False), label_dict=label_dict, images_dir="images", masks_dir="keypoint_masks")

    # split the dataset in train and test set
    indices = torch.randperm(len(dataset)).tolist()
    cutoff = int(len(dataset) * 0.8)
    dataset = torch.utils.data.Subset(dataset, indices[:cutoff])
    dataset_test = torch.utils.data.Subset(dataset_test, indices[cutoff:])

    # define training and validation data loaders
    data_loader = torch.utils.data.DataLoader(
        dataset,
        batch_size=params.hyperparams["batch_size"],
        shuffle=True,
        num_workers=params.hyperparams["num_workers"],
        collate_fn=utils.collate_fn,
    )

    data_loader_test = torch.utils.data.DataLoader(
        dataset_test,
        batch_size=2,
        shuffle=False,
        num_workers=params.hyperparams["num_workers"],
        collate_fn=utils.collate_fn,
    )

    # get the model using our helper function
    model = get_model_instance_segmentation(num_classes)

    # load weights to fine tune if applicable
    if init_weights_path is not None:
        model.load_state_dict(torch.load(init_weights_path))

    # move model to the right device
    model.to(device)

    # construct an optimizer
    optimizer_params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.SGD(
        optimizer_params,
        lr=params.hyperparams["learning_rate"],
        momentum=params.hyperparams["momentum"],
        weight_decay=params.hyperparams["weight_decay"],
    )
    # and a learning rate scheduler
    lr_scheduler = torch.optim.lr_scheduler.StepLR(
        optimizer,
        step_size=params.optimizer["step_size"],
        gamma=params.optimizer["gamma"],
    )

    # begin training
    num_epochs = n_epochs
    output_label = "mrcnn" if output_label is None else output_label
    for epoch in range(num_epochs):

        print("\n---- Training Model ----")
        model.train()
        lr_scheduler = None
        if epoch == 0:
            warmup_factor = 1.0 / 1000
            warmup_iters = min(1000, len(data_loader) - 1)

            lr_scheduler = utils.warmup_lr_scheduler(
                optimizer, warmup_iters, warmup_factor
            )

        for batch_i, (images, targets) in enumerate(
            tqdm.tqdm(data_loader, desc=f"Training Epoch {epoch}")
        ):
            images = list(image.to(device) for image in images)
            targets = [{k: v.to(device) for k, v in t.items()} for t in targets]

            loss_dict = model(images, targets)

            losses = sum(loss for loss in loss_dict.values())

            # reduce losses over all GPUs for logging purposes
            loss_dict_reduced = utils.reduce_dict(loss_dict)
            losses_reduced = sum(loss for loss in loss_dict_reduced.values())

            loss_value = losses_reduced.item()

            if not math.isfinite(loss_value):
                print("Loss is {}, stopping training".format(loss_value))
                print(loss_dict_reduced)
                sys.exit(1)

            optimizer.zero_grad()
            losses.backward()
            optimizer.step()

            if lr_scheduler is not None:
                lr_scheduler.step()

        if epoch % params.checkpoint_interval == 0 or epoch == num_epochs - 1:
            checkpoint_path = os.path.join(output_path, f"{output_label}_ckpt_{epoch}.pth")
            print(f"---- Saving checkpoint to: '{checkpoint_path}' ----")
            torch.save(model.state_dict(), checkpoint_path)
            evaluate(model, data_loader_test, device=device)

    print("That's it!")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--n_epochs", type=int, required=True)
    parser.add_argument("-d", "--dataset_path", type=str, required=True)
    parser.add_argument("-o", "--output_path", type=str, required=True)
    parser.add_argument("-l", "--output_label", type=str, default=None)
    parser.add_argument("-w", "--init_weights_path", type=str, default=None)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    main(
        n_epochs=args.n_epochs,
        dataset_path=args.dataset_path, 
        output_path=args.output_path,
        output_label=args.output_label,
        init_weights_path=args.init_weights_path)
