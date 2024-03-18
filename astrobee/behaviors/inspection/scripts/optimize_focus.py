#!/usr/bin/env python3
#
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


import argparse
import glob
import os
import re
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
from scipy.optimize import curve_fit
from sklearn.metrics import r2_score


def process_logs():
    log_files = glob.glob("*isaac_LOGCAT.logcat*")
    output_lines = []

    # Temporary variables to store context across lines
    in_take_picture_command = False
    timestamp = None
    haz_cam_distance = None
    calculated_focus_distance = None
    flash = True

    print(log_files)

    for log_file_path in log_files:
        with open(log_file_path, "r") as file:
            lines = file.readlines()

        for line in lines:
            # print(line)
            if "Received take picture command." in line:
                in_take_picture_command = True
            elif in_take_picture_command:

                focus_distance_match = re.search(r"Haz cam distance: (\d+\.\d+)", line)
                if focus_distance_match:
                    haz_cam_distance = focus_distance_match.group(1)

                focus_match = re.search(r"Calculated focus distance: (\d+\.\d+)", line)
                if focus_match:
                    focus_distance = focus_match.group(1)

                timestamp_match = re.search(
                    r"Writing image to file: /sdcard/data/gov.nasa.arc.irg.astrobee.sci_cam_image/delayed/(\d+\.\d+).jpg",
                    line,
                )
                if timestamp_match:
                    timestamp = timestamp_match.group(1)

                if "Camera focus distance successfully set to" in line:
                    flash = False

                if "Attempting to publish image!" in line:
                    if timestamp and haz_cam_distance and focus_distance:
                        if flash:
                            output_line = f": Scicam picture acquired - Timestamp: {timestamp}, Focus distance (m):{haz_cam_distance}, Focal distance :{focus_distance}\n"
                            output_lines.append(output_line)
                        else:
                            output_line = f": Scicam picture acquired - Timestamp: {timestamp}, Focus distance (m):{haz_cam_distance}, Focal distance :{focus_distance}, Flashlight: 0\n"
                            output_lines.append(output_line)

                    # Reset temporary variables
                    in_take_picture_command = False
                    timestamp = None
                    haz_cam_distance = None
                    calculated_focus_distance = None
                    flash = True

    with open("output.txt", "w") as output_file:
        output_file.writelines(output_lines)


def get_focus_from_log(directory):
    # print(files)
    # print(directory)
    file = open(directory + "_log", "r")
    lines = file.readlines()

    focus_distances_flash_on = {}
    focus_distances_flash_off = {}
    for line in lines:
        # Find the index of the substring
        index = line.find("Scicam picture acquired - Timestamp:")
        if index != -1:
            # Cut everything before the substring
            line = line[index:]
            parts = line.split(",")
            timestamp = float(parts[0].split(":")[1])
            focus_distance = float(parts[2].split(":")[1].strip())

            if (
                "Flashlight" in line
                and float(line.split("Flashlight:")[1].strip()) == 0
            ):
                focus_distances_flash_off[timestamp] = focus_distance
            else:
                focus_distances_flash_on[timestamp] = focus_distance

    return focus_distances_flash_on, focus_distances_flash_off


def calculate_sharpness_metric(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fm = cv2.Laplacian(gray, cv2.CV_64F).var()
    return fm


def select_roi(image, scale_percent=20):
    # Resize the image to reduce window size
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
    resized = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

    clone = resized.copy()
    (x, y, w, h) = cv2.selectROI(
        "Select ROI", clone, fromCenter=False, showCrosshair=True
    )
    cv2.destroyWindow("Select ROI")
    # Scale the ROI coordinates back to the original image size
    x, y, w, h = (
        int(x * (100 / scale_percent)),
        int(y * (100 / scale_percent)),
        int(w * (100 / scale_percent)),
        int(h * (100 / scale_percent)),
    )
    return (x, y, w, h)


# Define the function to fit the curve
def func(x, a, b):
    return a / x + b


def optimize_focus(args):
    directories = next(os.walk("."))[1]

    with PdfPages("focus_vs_sharpness.pdf") as pdf:
        # Store optimal values for each distance
        curve = {}
        directory_data = {}
        for directory in directories:
            print("Analysing " + directory)
            image_files = [
                file for file in os.listdir(directory) if file.endswith((".jpg"))
            ]

            # Select ROI for the first image only if args.all is not True
            if not args.all:
                roi = select_roi(cv2.imread(os.path.join(directory, image_files[0])))
                rois = [roi] * len(image_files)
            else:
                rois = [
                    select_roi(cv2.imread(os.path.join(directory, image_file)))
                    for image_file in image_files
                ]
            # Store image_files and (x, y, w, h) values in a dictionary
            directory_data[directory] = {"rois": rois}
        print(directory_data)
        # directory_data = {
        #     '025_bsharp': [{'roi': (2315, 1910, 1280, 1365)] * len(image_files)},
        #     '028_bsharp': [{'roi': (2175, 1935, 1125, 1210)] * len(image_files)},
        #     '030_bsharp': [{'roi': (2275, 1935, 1065, 1115)] * len(image_files)},
        #     '033_bsharp': [{'roi': (2605, 1945, 1000, 1040)] * len(image_files)},
        #     '035_bsharp': [{'roi': (2440, 1965, 920, 930)] * len(image_files)},
        #     '038_bsharp': [{'roi': (2265, 1960, 830, 865)] * len(image_files)},
        #     '040_bsharp': [{'roi': (2070, 1975, 825, 810)] * len(image_files)},
        #     '045_bsharp': [{'roi': (2415, 1965, 710, 740)] * len(image_files)},
        #     '050_bsharp': [{'roi': (2295, 1985, 620, 690)] * len(image_files)},
        #     '055_bsharp': [{'roi': (2710, 2010, 600, 575)] * len(image_files)},
        #     '060_bsharp': [{'roi': (2415, 1985, 545, 570)] * len(image_files)}
        # }
        # directory_data = {
        #     '025_wannabee': {'roi': [(2335, 1340, 1195, 1140)] * len(image_files)},
        #     '028_wannabee': {'roi': [(2055, 1420, 1080, 995)] * len(image_files)},
        #     '030_wannabee': {'roi': [(2195, 1470, 985, 930)] * len(image_files)},
        #     '033_wannabee': {'roi': [(2260, 1505, 905, 850)] * len(image_files)},
        #     '035_wannabee': {'roi': [(2295, 1540, 855, 805)] * len(image_files)},
        #     '038_wannabee': {'roi': [(2355, 1565, 820, 760)] * len(image_files)},
        #     '040_wannabee': {'roi': [(2415, 1615, 765, 705)] * len(image_files)},
        #     '045_wannabee': {'roi': [(2110, 1645, 675, 615)] * len(image_files)},
        #     '050_wannabee': {'roi': [(2350, 1690, 630, 575)] * len(image_files)},
        #     '055_wannabee': {'roi': [(2640, 1730, 535, 505)] * len(image_files)},
        #     '060_wannabee': {'roi': [(2475, 1755, 495, 455)] * len(image_files)}}
        # directory_data = {
        #     '025_honey': {'roi': [(1405, 610, 915, 1255)] * len(image_files)},
        #     '030_honey': {'roi': [(2125, 1450, 710, 1230)] * len(image_files)},
        #     '035_honey': {'roi': [(2055, 1560, 655, 1060)] * len(image_files)},
        #     '040_honey': {'roi': [(2295, 1580, 530, 875)] * len(image_files)}
        # }

        # Process images for each directory
        for directory, data in directory_data.items():
            image_files = [
                file for file in os.listdir(directory) if file.endswith((".jpg"))
            ]

            focus_distances_flash_on, focus_distances_flash_off = get_focus_from_log(
                directory
            )

            focus_distances_flash_on_values = []
            sharpness_values_flash_on = []
            focus_distances_flash_off_values = []
            sharpness_values_flash_off = []

            for roi, image_file in zip(data["rois"], image_files):
                (x, y, w, h) = roi

                if float(image_file.rsplit(".", 1)[0]) in focus_distances_flash_on:
                    focus_distances_flash_on_values.append(
                        focus_distances_flash_on[float(image_file.rsplit(".", 1)[0])]
                    )
                    sharpness_values_flash_on.append(
                        calculate_sharpness_metric(
                            cv2.imread(os.path.join(directory, image_file))[
                                y : y + h, x : x + w
                            ]
                        )
                    )

                if float(image_file.rsplit(".", 1)[0]) in focus_distances_flash_off:
                    focus_distances_flash_off_values.append(
                        focus_distances_flash_off[float(image_file.rsplit(".", 1)[0])]
                    )
                    sharpness_values_flash_off.append(
                        calculate_sharpness_metric(
                            cv2.imread(os.path.join(directory, image_file))[
                                y : y + h, x : x + w
                            ]
                        )
                    )

            max_sharpness_focus = focus_distances_flash_on_values[
                np.argmax(sharpness_values_flash_off)
            ]
            distance = (
                float(directory.split("_")[0]) / 100
            )  # Convert distance from cm to m
            robot = directory.split("_")[1]

            # plt.plot(focus_metrics_flash_on, 'bo', label='Flashlight On')
            plt.plot(
                focus_distances_flash_on_values,
                sharpness_values_flash_on,
                "bo",
                label="Flashlight On",
            )
            plt.plot(
                focus_distances_flash_off_values,
                sharpness_values_flash_off,
                "ro",
                label="Flashlight Off",
            )
            plt.xlabel("Focus value")
            plt.ylabel("Sharpness")
            plt.title("Focus vs. Sharpness for " + str(distance) + " in " + robot)
            plt.legend()
            pdf.savefig()
            plt.close()

            if robot not in curve:
                curve[robot] = []
            curve[robot].append([distance, max_sharpness_focus])
        print(curve)

    # curve = {'bsharp': [[0.25, 13.1082], [0.28, 10.6907], [0.3, 10.1367], [0.33, 8.34257], [0.35, 8.15647], [0.38, 7.32377], [0.4, 6.89125], [0.45, 4.90631], [0.5, 4.43838], [0.55, 3.71714], [0.6, 3.21222]]}
    # curve = {'wannabee': [[0.3, 13.1082], [0.33, 11.968], [0.35, 11.6251], [0.38, 10.6907], [0.4, 10.4074], [0.45, 8.73734], [0.5, 7.03046], [0.55, 6.75668], [0.6, 6.50063]]}
    # curve = {'honey': [[0.2, 10.690735], [0.226067, 11.968015], [0.280686, 9.59685], [0.345226, 7.32377]]}
    # curve = {'honey': [[0.226067, 11.968015], [0.280686, 9.59685], [0.345226, 7.32377]]}
    # curve = {'bumble': [[0.19208, 23.218346], [0.24541, 14.447436], [0.309338, 11.298598], [0.390805, 7.638627]]}

    with PdfPages("curve.pdf") as pdf:
        # Plotting
        for i, (robot, points) in enumerate(curve.items()):
            # Extract x and y values
            x_data, y_data = zip(*points)
            x_data = np.array(x_data)
            y_data = np.array(y_data)

            # Fit the curve
            popt, _ = curve_fit(func, x_data, y_data, maxfev=10000)

            # Calculate R^2 value
            y_pred = func(x_data, *popt)
            r2 = r2_score(y_data, y_pred)

            # Plot the points
            plt.scatter(x_data, y_data, label=robot)

            # Plot the fitted curve
            x_fit = np.linspace(min(x_data), max(x_data), 100)
            y_fit = func(x_fit, *popt)
            plt.plot(x_fit, y_fit, label=f"{robot} curve")

            # Add equation and R^2 value to the plot
            equation = (
                f"{robot} curve: y = {popt[0]:.2f} * x + {popt[1]:.2f}, R^2 = {r2:.2f}"
            )
            plt.annotate(
                equation,
                xy=(0.05, 0.95 - i * 0.1),
                xycoords="axes fraction",
                fontsize=10,
                ha="left",
                va="top",
            )

        # Add labels and legend
        plt.xlabel("X Axis Label")
        plt.ylabel("Y Axis Label")
        plt.legend()
        pdf.savefig(
            figsize=(8, 12)
        )  # Increase the figure size to accommodate the annotations
        plt.close()


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter):
    pass


def main():

    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )

    parser.add_argument(
        "--images_path",
        default=os.path.join(
            os.getenv("HOME"), "data/bags/sci_cam/20240222_bumble_cal"
        ),
        help="PDDL action name and its arguments",
    )
    parser.add_argument(
        "-g",
        "--granite",
        help="perform deserialization check on output bag (can be slow for large bags)",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "-l",
        "--logs",
        help="perform deserialization check on output bag (can be slow for large bags)",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "-a",
        "--all",
        help="select roi for all images",
        default=False,
        action="store_true",
    )

    args = parser.parse_args()

    print(args.images_path)
    os.chdir(args.images_path)

    if args.logs:
        process_logs()
    else:
        optimize_focus(args)


if __name__ == "__main__":
    sys.exit(main())
