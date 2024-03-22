#!/usr/bin/env python3

import ast
import csv
import json
import math
import os
import re
import subprocess
import sys
import time
import warnings

import craft.file_utils as file_utils
import cv2
import gdown
import image_str.net_utils as net_utils
import image_str.utils as utils
import IPython
import ipywidgets as widgets
import jellyfish
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.backends.cudnn as cudnn
import torch.nn as nn
from craft.craft import CRAFT
from IPython.display import HTML
from IPython.display import display as idisplay
from PIL import Image
from strhub.data.module import SceneTextDataModule
from torch.autograd import Variable
from tqdm import tqdm


class Ocr:
    def __init__(self, df=None, bag_path=None):
        self.net = self.__get_craft()
        self.parseq, self.img_transform = self.__get_parseq()

        if df is None:
            self.dataframe = pd.DataFrame(
                columns=[
                    "label",
                    "location",
                    "image",
                    "PCL Intersection",
                    "Mesh Intersection",
                ]
            )
        else:
            self.dataframe = df

        self.set_bag_path(bag_path)

    @staticmethod
    def df_from_file(file_path):
        """
        Generates a pandas dataframe from csv file with columns 'label', 'location' (location in image), 'image' (path to image),
        'PCL Intersection', and 'Mesh Intersection' to be used to find labels.

        @param file_path    path to csv file
        @returns dataframe  pandas dataframe with all the information from csv file
        """

        df = pd.read_csv(
            file_path,
            delimiter=";",
            skiprows=[1],
            usecols=[
                "label",
                "location",
                "image",
                "PCL Intersection",
                "Mesh Intersection",
            ],
            dtype=str,
            keep_default_na=False,
            na_values="",
        )

        def convert_to_rect(string):
            location = re.findall(r"[-+]?\d*\.\d+|\d+", string)
            location = [int(i) for i in location]
            return [[location[0], location[1]], [location[2], location[3]]]

        def convert_to_list(string):
            if pd.isna(string):
                return None
            nums = re.findall(r"[-+]?\d*\.\d+|\d+", string)
            return [float(i) for i in nums]

        df["location"] = df["location"].apply(convert_to_rect)
        df["PCL Intersection"] = df["PCL Intersection"].apply(convert_to_list)
        df["Mesh Intersection"] = df["Mesh Intersection"].apply(convert_to_list)

        ocr = Ocr(df=df)
        return ocr

    def parse_folder(
        self,
        image_folder,
        result_folder=None,
        increment=False,
        final_file=None,
    ):
        """
        @param image_folder     folder containing images to parse
        @param result_folder    if not None, folder to store resulting files
        @param increment        if True, stores the resulting files for each file in folder
        @param final_file       if None, creates a csv file to store parsed labels. If not None, adds to file provided.
        @returns                pandas dataframe with all the labels and info parsed from images in specified folder
        """

        all_locations = set()
        image_list, _, _ = file_utils.get_files(image_folder)

        if result_folder is not None and final_file is None:
            header = [
                "label",
                "PCL Intersection",
                "Mesh Intersection",
                "image",
                "location",
            ]
            final_file = result_folder + "all_locations.csv"
            f = open(final_file, "w")
            writer = csv.writer(f, delimiter=";")
            writer.writerow(header)
            f.close()

        for k, image_path in enumerate(tqdm(image_list, desc="Parsing through Images")):
            self.__decode_image(
                image_path,
                result_folder=result_folder,
                all_locations=all_locations,
                final_file=final_file,
                increment=increment,
            )

        print("Success")
        return self.dataframe

    def parse_image(
        self,
        image_file,
        result_folder=None,
        increment=False,
    ):
        """
        @param image_file       path to image to parse for text
        @param result_folder    if not None, folder to store resulting files
        @param increment        if True, stores the resulting files for each file in folder
        @returns                pandas dataframe with all the labels and info parsed from images in specified image file
        """

        tqdm(
            self.__decode_image(
                image_file,
                result_folder,
                final_file=None,
                increment=increment,
            )
        )

        print("Success")

    def __get_parseq(self):
        """
        @returns parseq models for OCR
        """

        # Load model and image transforms for parseq
        parseq = torch.hub.load("baudm/parseq", "parseq", pretrained=True).eval()
        img_transform = SceneTextDataModule.get_transform(parseq.hparams.img_size)
        return (parseq, img_transform)

    def __get_craft(self):
        """
        @returns craft models for OCR
        """

        cuda = False

        refiner_model = "weights/craft_refiner_CTW1500.pth"

        # load net
        net = CRAFT()  # initialize

        # Download craft model from web if does not exist in current path
        trained_model = "craft_mlt_25k.pth"
        if not os.path.exists(trained_model):
            url = "https://drive.google.com/uc?id=1Jk4eGD7crsqCCg9C9VjCLkMN3ze8kutZ"
            gdown.download(url, trained_model, quiet=False)
        else:
            print("Found Craft Model")

        if cuda:
            net.load_state_dict(newt_utils.copyStateDict(torch.load(trained_model)))
        else:
            net.load_state_dict(
                net_utils.copyStateDict(torch.load(trained_model, map_location="cpu"))
            )

        if cuda:
            net = net.cuda()
            net = torch.nn.DataParallel(net)
            cudnn.benchmark = False

        net.eval()

        return net

    def __get_bag_file(self, img_file):
        """
        Given an image, return the bag the image is from.

        @param img_file     the path to the ISS image
        @returns string     the name of the bag file the image originated from
        """

        filename, file_ext = os.path.splitext(os.path.basename(img_file))

        timestamp = float(filename)

        bumble_bag_files = {
            "20220711_1238_survey_usl_bay6_std_panorama_run1.bag": (
                1657543101.316589,
                1657543934.13,
            ),
            "20220711_1255_survey_usl_bay5_std_panorama_run_1.bag": (
                1657544120.136949,
                1657545770.15,
            ),
            "20220711_1426_survey_usl_bay4_std_panorama_run_1.bag": (
                1657549618.634919,
                1657551283.42,
            ),
        }

        queen_bag_files = {
            "20220711_1459_survey_usl_to_jem.bag": (1657551567.745524, 1657552130.3),
            "20220711_1123_survey_test.bag": (1657538626.193575, 1657538635.45),
            "20220711_1223_survey_jem_to_usl.bag": (1657542216.747706, 1657544398.74),
            "20220711_1259_survey_usl_bay3_std_panorama_run1.bag": (
                1657544400.135549,
                1657545492.46,
            ),
            "20220711_1318_survey_usl_bay2_std_panorama_run1.bag": (
                1657545494.376898,
                1657547254.04,
            ),
            "20220711_1358_survey_hatch_inspection_usl_fwd_run1.bag": (
                1657547939.494802,
                1657548383.24,
            ),
            "20220711_1433_survey_usl_bay1_std_panorama_run1.bag": (
                1657550015.257639,
                1657550898.37,
            ),
            "20220711_1452_survey_hatch_inspection_usl_fwd_close_run_1.bag": (
                1657551142.474949,
                1657551422.48,
            ),
            "20220711_1407_survey_usl_fwd_stereo_mapping_run_1.bag": (
                1657548455.916899,
                1657549490.51,
            ),
        }

        if "queen" in img_file:
            bag_files = queen_bag_files
        elif "bumble" in img_file:
            bag_files = bumble_bag_files
        else:
            raise Exception(
                "Unknown bag files. Is the image within a 'queen' or 'bumble' folder?"
            )

        for file in bag_files:
            if bag_files[file][0] <= timestamp and timestamp <= bag_files[file][1]:
                return file

        return None

    def __parse_3D_result(self, string):
        """
        Given the string of the output from find_point_coordinate, returns a dictionary of the values.

        @param string           output from find_point_coordinate
        @returns dictionary     parsed values from the string output
        """
        lines = string.split("\n")
        timestamps = re.findall(r"[-+]?\d*\.\d+|\d+", lines[0])
        vector = re.findall(r"[-+]?\d*\.\d+|\d+", lines[1])
        distance = re.findall(r"[-+]?\d*\.\d+|\d+", lines[2])
        pcl = re.findall(r"[-+]?\d*\.\d+|\d+", lines[3])
        pcl = [float(i) for i in pcl]
        mesh = re.findall(r"[-+]?\d*\.\d+|\d+", lines[4])
        mesh = [float(i) for i in mesh]
        results = {
            "Closest Timestamp Depth": float(timestamps[0]),
            "Closest Timestamp Pose": float(timestamps[1]),
            "Vector": tuple(float(i) for i in vector),
            "Point Cloud to Vector Distance": float(distance[0]),
            "PCL Intersection": None,
            "Mesh Intersection": None,
        }

        if len(pcl) != 0:
            results["PCL Intersection"] = {
                "x": pcl[0],
                "y": pcl[1],
                "z": pcl[2],
                "roll": pcl[3],
                "pitch": pcl[4],
                "yaw": pcl[5],
            }

        if len(mesh) != 0:
            results["Mesh Intersection"] = {
                "x": mesh[0],
                "y": mesh[1],
                "z": mesh[2],
                "roll": mesh[3],
                "pitch": mesh[4],
                "yaw": mesh[5],
            }

        return results

    def __decode_image(
        self,
        image_path,
        result_folder=None,
        all_locations=set(),
        final_file=None,
        increment=False,
    ):
        """
        @param image_path       path to image to parse
        @param result_folder    if provided will save the dataframe as a csv file to the folder
        @param all_locations    set to contain all the locations of labels found
        @param final_file       if provided will save all the labels and their corresponding info to the csv file
        @param increment        if true, will save the image with all the labels boxed and marked to result folder
        """

        if result_folder is not None and not os.path.isdir(result_folder):
            os.mkdir(result_folder)

        # ============================ Initialization ============================
        filename, file_ext = os.path.splitext(os.path.basename(image_path))

        result_path = None

        cuda = False
        refine = False
        poly = False
        # show_time = False
        text_threshold = 0.7
        low_text = 0.4
        link_threshold = 0.1
        # mag_ratio = 1.5
        refine_net = None

        # ============================ Start Processing ============================

        t = time.time()

        # load data
        image = cv2.imread(image_path)
        if image is None:
            raise Exception("Missing image file")

        h, w, _ = image.shape
        bag_name = None

        # Set up the command to find_point_coordinate
        if self.bag_path is not None:
            bag_name = self.__get_bag_file(image_path)
            if bag_name is not None:
                bag_name = self.bag_path + bag_name

                # Specify the ros command for 3D position
                json_file = "data.json"

                ros_command = [
                    "rosrun",
                    "pano_view",
                    "find_point_coordinate",
                    "--bag_name",
                    bag_name,
                    "--json_config",
                    json_file,
                ]
                data = {
                    "timestamp": float(filename),
                    "camera": "sci_cam",
                    "coord": {"x": 0, "y": 0},
                    "width": w,
                    "height": h,
                }

        # Set up for partial image cropping
        crop_w = 1500
        crop_h = 1500
        offset_w = 1000
        offset_h = 1000

        end_w = max(w - crop_w + offset_w, offset_w)
        end_h = max(h - crop_h + offset_h, offset_h)

        edge_border = 15
        total = round((end_w / offset_w) * (end_h / offset_h))
        num = 0

        # Run the text region detection on the entire image
        bboxes, polys, score_text = net_utils.test_net(
            self.net,
            image,
            text_threshold,
            link_threshold,
            low_text,
            cuda,
            poly,
            refine_net,
        )

        df = pd.DataFrame(columns=["label", "location"])

        for box in polys:
            # Box in form upper left -> upper right -> lower right -> lower left, (x, y)
            box = np.array(box).astype(np.int32)
            x_coordinates = box[:, 0]
            y_coordinates = box[:, 1]
            upper_left = np.array((min(x_coordinates), min(y_coordinates)))
            lower_right = np.array((max(x_coordinates), max(y_coordinates)))

            cropped_image = utils.crop_image(
                image, upper_left[0], upper_left[1], lower_right[0], lower_right[1]
            )

            new_img = Image.fromarray(np.array(cropped_image)).convert("RGB")
            new_img = self.img_transform(new_img).unsqueeze(0)

            logits = self.parseq(new_img)
            logits.shape  # torch.Size([1, 26, 95]), 94 characters + [EOS] symbol

            # Greedy decoding
            pred = logits.softmax(-1)
            label, confidence = self.parseq.tokenizer.decode(pred)

            new_location = np.array((upper_left, lower_right))

            # Check if detected region overlap with other regions already detected.
            overlap_result = df.loc[
                df["location"].apply(utils.overlap, args=(new_location,))
            ]

            if overlap_result.empty:
                df.loc[len(df)] = [label[0], np.array((upper_left, lower_right))]
            else:
                for index, row in overlap_result.iterrows():
                    old_label = row["label"]
                    old_location = row["location"]
                    if self.__similar(old_label, label[0]):
                        new_label = (
                            old_label if len(old_label) >= len(label[0]) else label[0]
                        )
                        new_location = utils.get_bounding_box(
                            old_location, new_location
                        )
                        new_row = np.array([new_label, new_location], dtype=object)
                        df.iloc[index] = new_row
        # Crop the image into sections to detect small text
        for x in range(0, end_w, offset_w):
            for y in range(0, end_h, offset_h):
                img = utils.crop_image(image, x, y, x + crop_w, y + crop_h)
                bboxes, polys, score_text = net_utils.test_net(
                    self.net,
                    img,
                    text_threshold,
                    link_threshold,
                    low_text,
                    cuda,
                    poly,
                    refine_net,
                )

                num += 1

                # # ============== parseq ============== #
                for box in polys:
                    # Box in form upper left -> upper right -> lower right -> lower left, (x, y)
                    box = np.array(box).astype(np.int32)
                    x_coordinates = box[:, 0]
                    y_coordinates = box[:, 1]
                    upper_left = np.array((min(x_coordinates), min(y_coordinates)))
                    lower_right = np.array((max(x_coordinates), max(y_coordinates)))
                    cropped_image = utils.crop_image(
                        img,
                        upper_left[0],
                        upper_left[1],
                        lower_right[0],
                        lower_right[1],
                    )

                    # Ignore labels near the edges of the cropped image
                    if (
                        min(x_coordinates) < edge_border
                        or max(x_coordinates) > crop_w - edge_border
                        or min(y_coordinates) < edge_border
                        or max(y_coordinates) > crop_h - edge_border
                    ):
                        continue
                    new_img = Image.fromarray(np.array(cropped_image)).convert("RGB")
                    new_img = self.img_transform(new_img).unsqueeze(0)

                    logits = self.parseq(new_img)
                    logits.shape  # torch.Size([1, 26, 95]), 94 characters + [EOS] symbol

                    # Greedy decoding
                    pred = logits.softmax(-1)
                    label, confidence = self.parseq.tokenizer.decode(pred)

                    # Convert to location on original image
                    upper_left = np.array((upper_left[0] + x, upper_left[1] + y))
                    lower_right = np.array((lower_right[0] + x, lower_right[1] + y))
                    new_location = np.array((upper_left, lower_right))
                    overlap_result = df.loc[
                        df["location"].apply(utils.overlap, args=(new_location,))
                    ]

                    if overlap_result.empty:
                        new_location = np.array((upper_left, lower_right))
                        index = len(df)
                        df.loc[index] = [label[0], new_location]
                    else:
                        for i, row in overlap_result.iterrows():
                            old_label = row["label"]
                            old_location = row["location"]
                            if self.__similar(old_label, label[0]):
                                new_label = (
                                    old_label
                                    if len(old_label) >= len(label[0])
                                    else label[0]
                                )
                                new_location = utils.get_bounding_box(
                                    old_location, new_location
                                )
                                new_row = np.array(
                                    [new_label, new_location], dtype=object
                                )

                                df.iloc[i] = new_row

        if result_folder is not None:
            result_path = result_folder + filename + ".jpg"

        if increment:
            result_image = self.__display_all(image, df, result_path)

        if bag_name is not None:
            # Get the locations in the world frame
            self.__get_all_locations(
                df,
                data,
                ros_command,
                all_locations,
                result_path,
                final_file,
                image_path,
                increment,
            )

    def __get_location(self, data, new_location, ros_command):
        """
        @param data         dictionary for all the parameters to be passed into find_point_coordinate
        @param new_location 2D list representing location of the label in the image (pixels)
        @param ros_command  list representing ros command used to call find_point_coordinate
        """

        data["coord"]["x"] = int((new_location[1][0] + new_location[0][0]) / 2)
        data["coord"]["y"] = int((new_location[1][1] + new_location[0][1]) / 2)

        json_file = ros_command[-1]

        with open(json_file, "w") as file:
            json.dump(data, file)

        # Run the ROS command using subprocess
        process = subprocess.Popen(
            ros_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )

        # Wait for the process to finish and capture the output
        stdout, stderr = process.communicate()
        stdout = stdout.decode()
        stderr = stderr.decode()

        # If error occured
        if len(stderr) != 0:
            return None

        result_positions = self.__parse_3D_result(stdout)

        return result_positions

    def __get_all_locations(
        self,
        image_df,
        data,
        ros_command,
        all_locations,
        result_path,
        final_file,
        image_path,
        increment,
    ):
        """
        Given parameters will save the labels and the corresponding info (pixel boundary box and 3D locations) from image_df to dataframe.

        @param image_df         pandas dataframe that stores all the labels found for specific image
        @param data             dictionary for all the parameters to be passed into find_point_coordinate
        @param ros_command      list representing ros command used to call find_point_coordinate
        @param all_locations    set to contain all the locations of labels found
        @param result_path      location to save all the found labels for specific image as csv file
        @param final_file       file that saves all the labels found for numerous images
        @param image_path       path to image
        @param increment        if true, will save the labels found to a csv file
        """

        total = len(image_df)
        header = ["label", "PCL Intersection", "Mesh Intersection", "image", "location"]
        f = None
        final = None
        if result_path is not None and increment:
            f = open(result_path[:-4] + "_locations.csv", "w")
            writer = csv.writer(f, delimiter=";")
            writer.writerow(header)

        if final_file is not None:
            final = open(final_file, "a")
            writer_final = csv.writer(final, delimiter=";")

        for i, row in image_df.iterrows():
            label = row["label"]
            new_location = row["location"]

            result_positions = self.__get_location(data, new_location, ros_command)
            if result_positions is None:
                continue
            location = tuple(result_positions["PCL Intersection"].values())
            duplicate = [loc for loc in all_locations if utils.duplicate(location, loc)]
            if len(duplicate) != 0:
                continue

            pcl = result_positions["PCL Intersection"]
            pcl_str = ""
            if pcl is not None:
                pcl = list(pcl.values())
                pcl_str = str(pcl)

            mesh = result_positions["Mesh Intersection"]
            mesh_str = ""
            if mesh is not None:
                mesh = list(mesh.values())
                mesh_str = str(mesh)

            self.dataframe.loc[len(self.dataframe)] = [
                label,
                new_location,
                image_path,
                pcl,
                mesh,
            ]

            line = np.array(
                [
                    label,
                    pcl_str,
                    mesh_str,
                    image_path,
                    str(new_location).replace("\n", ""),
                ]
            )
            if f is not None:
                writer.writerow(line)

            if final is not None:
                writer_final.writerow(line)

            all_locations.add(location)

        if f is not None:
            f.close()

        if final is not None:
            final.close()

    def __display_all(self, image, df, result_path=None):
        """
        @param image        openCV array of image
        @param df           pandas dataframe of columns "label" and "location" where "location" is the upper left
                            and lower right points of the rectangular bounding box for the corresponding label
        @param result_path  if provided will save the labeled image to the result folder.
        @returns array of image with the labels and boundary boxes displayed
        """

        display_image = image.copy()
        blue = (255, 0, 0)
        red = (0, 0, 255)
        thickness = 2
        font = cv2.FONT_HERSHEY_SIMPLEX
        result = ""
        for _, row in df.iterrows():
            rect = row["location"]
            display_image = cv2.rectangle(display_image, rect[0], rect[1], blue, 10)
            display_image = cv2.putText(
                display_image,
                row["label"],
                rect[0],
                font,
                1,
                red,
                thickness,
                cv2.LINE_AA,
            )
            result += row["label"] + " " + str(row["location"]) + "\n"

        if result_path is not None:
            cv2.imwrite(result_path, display_image)
            with open(result_path[:-4] + ".txt", "w") as text_file:
                text_file.write(result)

        return display_image

    def __similar(self, label, input_label):
        """
        Returns true is label and input_label are similar.

        @param label        string
        @param input_label  string
        @returns true if label and input_label is within a similarity threshold
        """

        # [0, 1] where 0 represents two completely dissimilar strings and 1 represents identical strings
        label = label.upper()
        input_label = input_label.upper()
        return jellyfish.jaro_winkler_similarity(label, input_label) > 0.8

    def set_bag_path(self, bag):
        """
        Sets the path to the bag files to be used in finding location of the label in the world frame.

        @param bag  string to folder with all the bag files
        """

        self.bag_path = bag

    def find_label(self, label, display_img=True, jupyter=False):
        """
        Search for specified label and if display_img is true, create an interactive graph for Jupyter Notebook
        display.

        @param label        string to search for in dataframe
        @param display_img  if True, display images with label boxed and cropped.
        @param jupyter      True if program is ran in Jupyter Notebook (necessary for interactive display)
        @throws error       if display_img is True, but the image path specified in the dataframe does not exist.
        """

        # Sort such that labels closest to search label appear first.
        self.dataframe["similarity"] = self.dataframe["label"].apply(
            jellyfish.jaro_winkler_similarity, args=(label,)
        )
        self.dataframe.sort_values(by="similarity", ascending=False, inplace=True)
        self.dataframe.drop(columns="similarity", inplace=True)

        # Get all images with label
        words = label.split()
        l_result = self.dataframe.loc[
            self.dataframe["label"].apply(self.__similar, args=(words[0],))
        ]

        # Search through one image at a time
        images = set(l_result["image"].tolist())
        for l in words[1:]:
            l_result = self.dataframe.loc[
                self.dataframe["label"].apply(self.__similar, args=(l,))
            ]
            images = images.intersection(set(l_result["image"].tolist()))

        rectangles = []
        results = []
        full = []
        for img_file in tqdm(images, desc="Searching for {:s}".format(label)):
            df = self.dataframe.loc[self.dataframe["image"] == img_file]
            result = self.__find_image(img_file, df, label, display_img)
            full.extend(result[0])
            rectangles.extend(result[1])
            results.extend(result[2])

        if jupyter:
            # Text box to display 3D location of labels
            result_input = widgets.Textarea(
                disable=True,
                layout=widgets.Layout(height="100%", width="100%"),
            )

        def display_labels(fig, image_file, rect, result, title):
            offset = 100
            if display_img:
                image = cv2.imread(image_file)
                if image is None:
                    raise Exception("Missing image from path")

                full = cv2.rectangle(
                    image,
                    [i - offset for i in rect[0]],
                    [i + offset for i in rect[1]],
                    (255, 0, 0),
                    10,
                )

                cropped = utils.crop_image(
                    image,
                    rect[0][0] - offset,
                    rect[0][1] - offset,
                    rect[1][0] + offset,
                    rect[1][1] + offset,
                )

                fig.axes[0].imshow(cv2.cvtColor(full, cv2.COLOR_BGR2RGB))
                fig.axes[0].axis("off")

                im = fig.axes[1].imshow(cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB))
                fig.axes[1].axis("off")

                plt.draw()

            if jupyter:
                result_input.value = title + "\n" + result
                plt.title(title)
            else:
                cls = lambda: os.system("cls" if os.name == "nt" else "clear")
                cls()
                print(title + "\n" + result)

        index = 0

        def callback_left_button(event):
            nonlocal index, full, rectangles, results
            if index == 0:
                return
            index -= 1
            title = "Image: {:d}/{:d}".format(index + 1, len(results))
            if display_img:
                fig = plt.gcf()
                display_labels(
                    fig, full[index], rectangles[index], results[index], title
                )
            else:
                display_labels(None, None, None, results[index], title)

        def callback_right_button(event):
            nonlocal index, full, rectangles, results
            """ this function gets called if we hit the left button"""
            if index >= len(results) - 1:
                return
            index += 1
            title = "Image: {:d}/{:d}".format(index + 1, len(results))
            if display_img:
                fig = plt.gcf()
                display_labels(
                    fig, full[index], rectangles[index], results[index], title
                )
            else:
                display_labels(None, None, None, results[index], title)

        if len(results) == 0:
            print("No results found")
            return

        title = "Image: {:d}/{:d}".format(index + 1, len(results))
        if display_img:
            fig, ax = plt.subplots(1, 2, figsize=(10, 5))
            plt.tight_layout()
            display_labels(fig, full[0], rectangles[0], results[0], title)
        else:
            display_labels(None, None, None, results[0], title)

        if jupyter:
            # Update panorama link
            def update_link(result):
                if len(result) == 0:
                    return
                new_url = result.split("\n")[1]
                link_html = f'<a href="{new_url}" target="_blank">{new_url}</a>'
                display(HTML(link_html))

            previous = widgets.Button(
                description="Prev",
            )

            previous.on_click(callback_left_button)
            display(previous)
            right = widgets.Button(
                description="Next",
            )

            display(right)
            right.on_click(callback_right_button)

            # Update panorama link with each text box update
            widgets.interact(update_link, result=result_input)
            if display_img:
                plt.show()
        else:
            # Define custom button positions
            button_back = plt.axes([0.1, 0.01, 0.1, 0.05])  # [x, y, width, height]
            button_forward = plt.axes([0.2, 0.01, 0.1, 0.05])

            # Create custom buttons
            btn_back = plt.Button(button_back, "Back")
            btn_forward = plt.Button(button_forward, "Forward")

            btn_back.on_clicked(callback_left_button)
            btn_forward.on_clicked(callback_right_button)
            plt.show()

    def __find_image(self, image_file, df, label, display_img):
        """
        @param image_file   path to image currently being searched
        @param df           dataframe for all labels in specified image
        @param label        label to be searched
        @param display_img  if True, display images with label boxed and cropped.
        @returns            images array that holds images file paths for each rectangle.
                            rects array that holds rectangle positions of each label
                            results array that holds the 3D location of the labels in the panorama/ISS
                            if display_img is False, full and crop will always be empty.
        @throws error       if display_img is True, but the image path specified in the dataframe does not exist.
        """
        words = label.split()  # if search label is multi-word
        results = {}
        for l in words:
            # Find all rectangles for each word
            l_result = df.loc[df["label"].apply(self.__similar, args=(l,))]
            results[l] = l_result

        # Search for multi-word labels. Merge rectangles close to each other
        positions = np.array(results[words[0]]["PCL Intersection"].tolist())
        rectangles = np.array(results[words[0]]["location"].tolist())
        for word in words[1:]:
            new_rects = []
            new_positions = []
            other_rects = results[word]["location"].tolist()
            other_positions = results[word]["PCL Intersection"].tolist()
            if None in other_positions:
                other_positions = None

            for j in range(len(rectangles)):
                limit = 2 * (rectangles[j][1][1] - rectangles[j][0][1])
                closest_rect, mid_positions = utils.get_closest_rect(
                    rectangles[j], other_rects, other_positions, limit
                )
                for i in range(len(closest_rect)):
                    new_rects.append(
                        utils.get_bounding_box(rectangles[j], closest_rect[i])
                    )
                    if mid_positions is not None:
                        new_positions.append(
                            utils.get_midpoint(positions[j], mid_positions[i])
                        )
                    else:
                        new_positions = None
            rectangles = new_rects
            positions = new_positions

        cropped_images = []
        locations = []
        all_locations = set()
        rects = []
        for i in range(len(rectangles)):
            pos = positions[i]
            duplicate = [loc for loc in all_locations if utils.duplicate(pos, loc)]

            # Ignore duplicates
            if len(duplicate) != 0:
                continue

            rects.append(rectangles[i])
            all_locations.add(tuple(pos))

            # Create panorama link
            pitch = pos[4]
            yaw = pos[5]
            bag = self.__get_bag_file(image_file)
            if bag is None:
                print("None bag", image_file)
            if "bay1" in bag:
                loc = "usl_bay1"
            elif "bay2" in bag:
                loc = "usl_bay2"
                continue  # Panorama not available
            elif "bay3" in bag:
                loc = "usl_bay3"
                continue  # Panorama not available
            elif "bay4" in bag:
                loc = "usl_bay4"
            elif "bay5" in bag:
                loc = "usl_bay5"
            elif "bay6" in bag:
                loc = "usl_bay6"
            else:
                loc = ""

            link = "https://ivr.ndc.nasa.gov/isaac_panos/pannellum.htm?config=tour.json&firstScene={:s}&pitch={:f}&yaw={:f}&hfov=30\n".format(
                loc, -pitch, yaw
            )

            loc = (
                "Position (x, y, z): {:s}\nOrientation (roll, pitch, yaw): {:s}".format(
                    str(pos[:3]), str(pos[3:])
                )
            )

            result = link + loc
            locations.append(result)

        images = []
        if display_img:
            images = [image_file for _ in rects]

        return images, rects, locations


if __name__ == "__main__":
    warnings.filterwarnings("ignore")
    os.environ["ASTROBEE_CONFIG_DIR"] = "/home/rlu3/astrobee/src/astrobee/config"
    os.environ["ASTROBEE_RESOURCE_DIR"] = "/home/rlu3/astrobee/src/astrobee/resources"
    os.environ["ASTROBEE_ROBOT"] = "queen"
    os.environ["ASTROBEE_WORLD"] = "iss"

    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bag_path", help="Path to bag folder where the images came from."
    )
    parser.add_argument("--image_file", help="Path to image to parse.")
    parser.add_argument("--image_folder", help="Path to image folder to parse images.")
    parser.add_argument(
        "--result_folder", help="Path to result folder to save results."
    )
    parser.add_argument(
        "--increment",
        help="If True, will save the results of each individual image.",
        type=bool,
        default=True,
    )
    parser.add_argument(
        "--df_file", help="If provided, will create an ocr using data from csv file."
    )

    args = parser.parse_args()
    d = vars(args)

    bag_path = d["bag_path"]
    if d["df_file"] is not None:
        ocr = Ocr.df_from_file(d["df_file"])
        print("Created ocr. Use ocr.find_label(label, display) to search for labels.\n")
    else:
        ocr = Ocr(bag_path=bag_path)
        if d["image_file"] is not None:
            ocr.parse_image(
                d["image_file"],
                result_folder=d["result_folder"],
                increment=d["increment"],
            )
            print(
                "Created ocr. Finished parsing image. Use ocr.find_label(label, display) to search for labels.\n"
            )
        elif d["image_folder"] is not None:
            ocr.parse_folder(
                d["image_folder"],
                result_folder=d["result_folder"],
                increment=d["increment"],
            )
            print(
                "Created ocr. Finished parsing folder. Use ocr.find_label(label, display) to search for labels.\n"
            )

    """
    Examples:
    
    test_image = "/srv/novus_1/mgouveia/data/bags/20220711_Isaac11/queen/isaac_sci_cam_image_delayed/1657551170.616.jpg"
    result_folder = "result/test/"
    bag_path = "/srv/novus_1/mgouveia/data/bags/20220711_Isaac11/queen/"
    test_folder = "/srv/novus_1/mgouveia/data/bags/20220711_Isaac11/queen/isaac_sci_cam_image_delayed/"
    ocr = Ocr(bag_path=bag_path)
    ocr.parse_image(test_image, result_folder=result_folder, increment=True)
    ocr.parse_folder(test_folder, result_folder=result_folder, increment=True)
    ocr = Ocr.df_from_file(
        "/home/rlu3/isaac/src/anomaly/image_str/scripts/image_str/result/beehive/queen/all_locations.csv"
    )
    """

    IPython.embed()
