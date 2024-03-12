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


# Third party imports
import torch
import torchvision


def get_model(num_classes, model_name, weights_path=None):

    if model_name == "FASTERRCNN_MOBILENET_V3_LARGE_320_FPN":

        # download pretrained model with three out of six backbone layers set to be trainable (default)
        model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn(weights="DEFAULT", weights_backbone="DEFAULT")

        # we need to replace the box predictor head with a new one,
        # since we're training it from scratch for a different number of output classes
        in_features = model.roi_heads.box_predictor.cls_score.in_features
        model.roi_heads.box_predictor = torchvision.models.detection.faster_rcnn.FastRCNNPredictor(in_features, num_classes)

    elif model_name == "FASTERRCNN_MOBILENET_V3_LARGE_FPN":

        # download pretrained model with three out of six backbone layers set to be trainable (default)
        model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_fpn(weights="DEFAULT", weights_backbone="DEFAULT")

        # we need to replace the box predictor head with a new one,
        # since we're training it from scratch for a different number of output classes
        in_features = model.roi_heads.box_predictor.cls_score.in_features
        model.roi_heads.box_predictor = torchvision.models.detection.faster_rcnn.FastRCNNPredictor(in_features, num_classes)

    else:
        raise ValueError(f"Unknown model {model} specified.")
    
    # load weights if weights were provided
    if weights_path is not None:
        model.load_state_dict(torch.load(weights_path))
    
    return model
    