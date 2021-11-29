# Copyright (c) 2021, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
# platform" software is licensed under the Apache License, Version 2.0
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

import torch
from torch import nn
from torch import optim
import torch.nn.functional as F
from torchvision import datasets, transforms, models
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from PIL import Image

import torchvision.transforms as transforms

# Parameters
image = Image.open("./analysed_images/img3.jpg")

test_transforms = transforms.Compose([transforms.Resize(224),
                                      transforms.ToTensor(),
                                      transforms.Normalize([0.485, 0.456, 0.406],
                                                           [0.229, 0.224, 0.225])])
image_tensor = test_transforms(image).float()
image_tensor = image_tensor.unsqueeze_(0)

print(image_tensor[0, :5])


model = models.densenet121(pretrained=True)
model.classifier = nn.Sequential(nn.Linear(1024, 256),
                                 nn.ReLU(),
                                 nn.Dropout(0.2),
                                 nn.Linear(256, 3),
                                 nn.LogSoftmax(dim=1))
model.load_state_dict(torch.load('model_cnn.pt'))
model.eval()

output = model(image_tensor)
print(output)

if output is 0:
	print("Vent is obstructed")
else if output is 1:
	print("Vent is free")
else if output is 2:
	print("Could not identify a vent")

index = output.data.numpy().argmax()
print(index)

# 0 : -0.3883 -2.1018 -1.6115
# 1 : -0.3694 -1.6222 -2.1950
# 2 : -0.029524 -3.546348 -8.242915
# 3 : -0.9386 -0.5036 -5.4123