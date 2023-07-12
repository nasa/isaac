# Copyright © 2021, United States Government, as represented by the Administrator of the
# National Aeronautics and Space Administration. All rights reserved.
#
# The “ISAAC - Integrated System for Autonomous and Adaptive Caretaking platform” software is
# licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed under the
# License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific language governing
# permissions and limitations under the License.
#

import cv2
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


def plot_query_result(result, max_images, num_columns, img_size=20):
    # Create a subplot grid based on the number of images
    num_images = min(max_images, len(result))
    num_rows = (num_images - 1) // num_columns + 1

    # Create the figure and subplots using GridSpec
    fig = plt.figure(figsize=(img_size, img_size))
    gs = GridSpec(num_rows, num_columns, figure=fig)

    for idx, element in enumerate(result):
        if idx >= max_images:
            break
        image_path = "data/sci_cam_images/" + str(element["img"])
        image = cv2.imread(image_path)

        # Define the vertices for the lines
        vertices = [
            element["coord_c1"],
            element["coord_c2"],
            element["coord_c3"],
            element["coord_c4"],
        ]

        # Draw lines on the image
        for i in range(len(vertices) - 1):
            cv2.line(image, vertices[i], vertices[i + 1], (0, 0, 255), thickness=20)
        cv2.line(
            image, vertices[0], vertices[len(vertices) - 1], (0, 0, 255), thickness=20
        )

        # Convert BGR image to RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Create a new subplot within the GridSpec and plot the image
        ax = fig.add_subplot(gs[idx // num_columns, idx % num_columns])
        ax.imshow(image_rgb)
        ax.axis("off")

    # Adjust the spacing between subplots
    plt.tight_layout()

    # Show the plot
    plt.show()
