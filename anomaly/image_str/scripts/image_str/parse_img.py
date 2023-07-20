import ast
import csv
import json
import math
import os
import re
import subprocess
import time
import warnings
from multiprocessing import Process

import craft.file_utils as file_utils
import cv2
import image_str.net_utils as net_utils
import image_str.utils as utils
import IPython
import jellyfish
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.backends.cudnn as cudnn
import torch.nn as nn
from craft.craft import CRAFT
from parseq.strhub.data.module import SceneTextDataModule
from PIL import Image
from torch.autograd import Variable
from tqdm import tqdm


def get_bag_file(timestamp):
    bag_files = {
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

    for file in bag_files:
        if bag_files[file][0] <= timestamp and timestamp <= bag_files[file][1]:
            return file

    return None


def parse_3D_result(string):
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


def decode_image(
    image_path,
    database,
    result_folder=None,
    all_locations=set(),
    final_file=None,
    net=None,
    parseq_model=None,
    increment=False,
):
    """
    @param image_path path to image to parse
    @param result_folder if provided will save the marked image with boundary boxes and labels to the folder
    @param trained_model path to trained model
    @returns database of labels database["label"] and boundary boxes database["location"]
             result_image array representing labeled image
             image array representing original nonlabeled image
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

    parseq, img_transform = parseq_model

    # ============================ Start Processing ============================

    t = time.time()

    # load data
    image = cv2.imread(image_path)

    h, w, _ = image.shape
    bag_name = None

    if bag_path is not None:
        bag_name = get_bag_file(float(filename))
        if bag_name is not None:
            bag_name = bag_path + bag_name

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

    crop_w = 1500
    crop_h = 1500
    offset_w = 1000
    offset_h = 1000

    end_w = max(w - crop_w + offset_w, offset_w)
    end_h = max(h - crop_h + offset_h, offset_h)

    edge_border = 15
    total = round((end_w / offset_w) * (end_h / offset_h))
    num = 0

    bboxes, polys, score_text = net_utils.test_net(
        net,
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
        new_img = img_transform(new_img).unsqueeze(0)

        logits = parseq(new_img)
        logits.shape  # torch.Size([1, 26, 95]), 94 characters + [EOS] symbol

        # Greedy decoding
        pred = logits.softmax(-1)
        label, confidence = parseq.tokenizer.decode(pred)

        new_location = np.array((upper_left, lower_right))
        overlap_result = df.loc[
            df["location"].apply(utils.overlap, args=(new_location,))
        ]

        if overlap_result.empty:
            df.loc[len(df)] = [label[0], np.array((upper_left, lower_right))]
        else:
            for index, row in overlap_result.iterrows():
                old_label = row["label"]
                old_location = row["location"]
                if similar(old_label, label[0]):
                    new_label = (
                        old_label if len(old_label) >= len(label[0]) else label[0]
                    )
                    new_location = utils.get_bounding_box(old_location, new_location)
                    new_row = np.array([new_label, new_location], dtype=object)
                    df.iloc[index] = new_row

    for x in range(0, end_w, offset_w):
        for y in range(0, end_h, offset_h):
            img = utils.crop_image(image, x, y, x + crop_w, y + crop_h)
            bboxes, polys, score_text = net_utils.test_net(
                net,
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
                    img, upper_left[0], upper_left[1], lower_right[0], lower_right[1]
                )
                if (
                    min(x_coordinates) < edge_border
                    or max(x_coordinates) > crop_w - edge_border
                    or min(y_coordinates) < edge_border
                    or max(y_coordinates) > crop_h - edge_border
                ):
                    continue
                new_img = Image.fromarray(np.array(cropped_image)).convert("RGB")
                new_img = img_transform(new_img).unsqueeze(0)

                logits = parseq(new_img)
                logits.shape  # torch.Size([1, 26, 95]), 94 characters + [EOS] symbol

                # Greedy decoding
                pred = logits.softmax(-1)
                label, confidence = parseq.tokenizer.decode(pred)

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
                        if similar(old_label, label[0]):
                            new_label = (
                                old_label
                                if len(old_label) >= len(label[0])
                                else label[0]
                            )
                            new_location = utils.get_bounding_box(
                                old_location, new_location
                            )
                            new_row = np.array([new_label, new_location], dtype=object)

                            df.iloc[i] = new_row

    if result_folder is not None:
        result_path = result_folder + filename + ".jpg"

    if increment:
        result_image = display_all(image, df, result_path)
    if bag_name is not None:
        get_all_locations(
            database,
            df,
            data,
            bag_name,
            ros_command,
            all_locations,
            result_path,
            final_file,
            image_path,
        )


def get_location(data, new_location, ros_command):
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
    if len(stderr) != 0:
        return None

    result_positions = parse_3D_result(stdout)

    return result_positions


def get_all_locations(
    database,
    image_df,
    data,
    bag_name,
    ros_command,
    all_locations,
    result_path,
    final_file,
    image_file,
):
    total = len(image_df)
    header = ["label", "PCL Intersection", "Mesh Intersection"]
    f = None
    final = None
    if result_path is not None:
        f = open(result_path[:-4] + "_locations.csv", "w")
        writer = csv.writer(f, delimiter=";")
        writer.writerow(header)

    if final_file is not None:
        final = open(final_file, "a")
        writer_final = csv.writer(final, delimiter=";")

    for i, row in image_df.iterrows():
        label = row["label"]
        new_location = row["location"]

        result_positions = get_location(data, new_location, ros_command)
        if result_positions is None:
            continue

        location = tuple(result_positions["PCL Intersection"].values())
        if location in all_locations:
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

        database[len(database)] = [label, new_location, image_file, pcl, mesh]

        line = np.array(
            [label, pcl_str, mesh_str, image_file, str(new_location).replace("\n", "")]
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


def display_all(image, database, result_path=None):
    """
    @param image    openCV array of image
    @param database pandas database of columns "label" and "location" where "location" is the upper left
                    and lower right points of the rectangular bounding box for the corresponding label
    @returns array of image with the labels and boundary boxes displayed
    """
    display_image = image.copy()
    blue = (255, 0, 0)
    red = (0, 0, 255)
    thickness = 2
    font = cv2.FONT_HERSHEY_SIMPLEX
    result = ""
    for _, row in database.iterrows():
        rect = row["location"]
        display_image = cv2.rectangle(display_image, rect[0], rect[1], blue, 10)
        display_image = cv2.putText(
            display_image, row["label"], rect[0], font, 1, red, thickness, cv2.LINE_AA
        )
        result += row["label"] + " " + str(row["location"]) + "\n"

    if result_path is not None:
        cv2.imwrite(result_path, display_image)
        with open(result_path[:-4] + ".txt", "w") as text_file:
            text_file.write(result)

    return display_image


def similar(label, input_label):
    # [0, 1] where 0 represents two completely dissimilar strings and 1 represents identical strings
    label = label.upper()
    input_label = input_label.upper()
    return jellyfish.jaro_winkler_similarity(label, input_label) > 0.8


def df_from_file(file_path):
    df = pd.read_csv(
        file_path,
        delimiter=";",
        skiprows=[1],
        usecols=["label", "location", "image", "PCL Intersection", "Mesh Intersection"],
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
    return df


def set_bag_path(bag):
    global bag_path
    bag_path = bag


def find_panorama(database, label):
    words = label.split()
    l_result = database.loc[database["label"].apply(similar, args=(words[0],))]
    images = set(l_result["image"].tolist())
    for l in words[1:]:
        l_result = database.loc[database["label"].apply(similar, args=(l,))]
        images = images.intersection(set(l_result["image"].tolist()))
    full = []
    crop = []
    results = []

    for img_file in tqdm(images, desc="Searching for {:s}".format(label)):
        df = database.loc[database["image"] == img_file]
        result = find_image(img_file, df, label)
        full.extend(result[0])
        crop.extend(result[1])
        if result[2] is not None:
            results.extend(result[2])

    for i in range(len(results)):
        print(
            "Location {:d}\n Position (x, y, z): {:s}\n Orientation (roll, pitch, yaw): {:s}\n".format(
                i, str(results[i][:3]), str(results[i][3:])
            )
        )

    plt.show()
    return full, crop, result


def find_image(image_file, database, label):
    """
    @param image    array representing image to be searched
    @param database database of labels (database["labels"]) and their corresponding boundary boxes (database["location"])
    @param label text to search for
    @returns array representing image with label boxed if found and a list of all labels cropped from original image
    """
    image = cv2.imread(image_file)
    words = label.split()
    results = {}
    for l in words:
        l_result = database.loc[database["label"].apply(similar, args=(l,))]
        results[l] = l_result

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
                new_rects.append(utils.get_bounding_box(rectangles[j], closest_rect[i]))
                if mid_positions is not None:
                    new_positions.append(
                        utils.get_midpoint(positions[j], mid_positions[i])
                    )
                else:
                    new_positions = None

        rectangles = new_rects
        positions = new_positions

    new_image = np.array(image)

    offset = 10
    cropped_images = []
    for i in range(len(rectangles)):
        new_image = cv2.rectangle(
            new_image, rectangles[i][0], rectangles[i][1], (255, 0, 0), 10
        )
        cropped_images.append(
            utils.crop_image(
                image,
                rectangles[i][0][0] - offset,
                rectangles[i][0][1] - offset,
                rectangles[i][1][0] + offset,
                rectangles[i][1][1] + offset,
            )
        )

    for i in range(0, len(cropped_images), 2):
        fig = plt.figure()
        ax1 = fig.add_subplot(122)
        ax1.imshow(new_image)

        ax2 = fig.add_subplot(221)
        ax2.imshow(cropped_images[i])
        if i + 1 == len(cropped_images):
            break
        ax3 = fig.add_subplot(223)
        ax3.imshow(cropped_images[i + 1])

    if len(cropped_images) == 0:
        new_image = []
    else:
        new_image = [new_image]

    return new_image, cropped_images, positions


def display_images(images):
    fig = None
    size = 2
    for i, image in enumerate(images):
        if i % 4 == 0:
            fig = plt.figure()
        fig.add_subplot(size, size, i % 4 + 1)
        plt.imshow(image)

    plt.show()


def get_parseq():
    # Load model and image transforms for parseq
    parseq = torch.hub.load("baudm/parseq", "parseq", pretrained=True).eval()
    img_transform = SceneTextDataModule.get_transform(parseq.hparams.img_size)
    return (parseq, img_transform)


def get_craft(trained_model):
    cuda = False

    refiner_model = "weights/craft_refiner_CTW1500.pth"
    # trained_model = "models/craft_mlt_25k.pth"

    # load net
    net = CRAFT()  # initialize

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


def parse_folder(image_folder, trained_model=None, result_folder=None, increment=False):
    database = pd.DataFrame(
        columns=["label", "location", "image", "PCL Intersection", "Mesh Intersection"]
    )
    all_locations = set()
    image_list, _, _ = file_utils.get_files(image_folder)

    net = get_craft(trained_model)
    parseq = get_parseq()

    final_file = None
    if result_folder is not None:
        header = ["label", "PCL Intersection", "Mesh Intersection", "image", "location"]
        final_file = result_folder + "all_locations.csv"
        f = open(final_file, "w")
        writer = csv.writer(f, delimiter=";")
        writer.writerow(header)
        f.close()

    for k, image_path in enumerate(tqdm(image_list, desc="Parsing through Images")):
        decode_image(
            image_path,
            database,
            result_folder=result_folder,
            all_locations=all_locations,
            final_file=final_file,
            net=net,
            parseq_model=parseq,
            increment=increment,
        )

    print("Success")
    return database


def parse_image(image_file, trained_model=None, result_folder=None, increment=False):
    database = pd.DataFrame(
        columns=["label", "location", "image", "PCL Intersection", "Mesh Intersection"]
    )

    net = get_craft(trained_model)
    parseq = get_parseq()
    tqdm(
        decode_image(
            image_file,
            database,
            result_folder,
            final_file=None,
            net=net,
            parseq_model=parseq,
            increment=increment,
        )
    )

    print("Success")
    return database


if __name__ == "__main__":
    warnings.filterwarnings("ignore")
    os.environ["ASTROBEE_CONFIG_DIR"] = "/home/rlu3/astrobee/src/astrobee/config"
    os.environ["ASTROBEE_RESOURCE_DIR"] = "/home/rlu3/astrobee/src/astrobee/resources"
    os.environ["ASTROBEE_ROBOT"] = "queen"
    os.environ["ASTROBEE_WORLD"] = "iss"

    test_image = "/srv/novus_1/mgouveia/data/bags/20220711_Isaac11/queen/isaac_sci_cam_image_delayed/1657545020.689.jpg"
    result_folder = "result/beehive/queen/"
    bag_path = "/srv/novus_1/mgouveia/data/bags/20220711_Isaac11/queen/"
    test_folder = "/srv/novus_1/mgouveia/data/bags/20220711_Isaac11/queen/isaac_sci_cam_image_delayed/"
    set_bag_path(bag_path)
    # database = parse_folder(test_folder, result_folder=result_folder)
    # database = parse_image(test_image, bad_path, result_folder)
    database = df_from_file(
        "/home/rlu3/isaac/src/anomaly/image_str/scripts/image_str/result/beehive/queen/all_locations.csv"
    )

    IPython.embed()
