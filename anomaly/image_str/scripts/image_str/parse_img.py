import json
import os
import subprocess
import time
from collections import OrderedDict

import warnings
import re

import craft.craft_utils as craft_utils
import craft.file_utils as file_utils
import craft.imgproc as imgproc
import cv2
import image_str.utils as utils
import IPython
import jellyfish
import numpy as np
import pandas as pd
import torch
import torch.backends.cudnn as cudnn
import torch.nn as nn
from craft.craft import CRAFT
from parseq.strhub.data.module import SceneTextDataModule
from PIL import Image
from torch.autograd import Variable


def copyStateDict(state_dict):
    if list(state_dict.keys())[0].startswith("module"):
        start_idx = 1
    else:
        start_idx = 0
    new_state_dict = OrderedDict()
    for k, v in state_dict.items():
        name = ".".join(k.split(".")[start_idx:])
        new_state_dict[name] = v
    return new_state_dict


def test_net(
    net, image, text_threshold, link_threshold, low_text, cuda, poly, refine_net=None
):
    t0 = time.time()

    canvas_size = 1280
    mag_ratio = 1.5

    # resize
    img_resized, target_ratio, size_heatmap = imgproc.resize_aspect_ratio(
        image, canvas_size, interpolation=cv2.INTER_LINEAR, mag_ratio=mag_ratio
    )
    ratio_h = ratio_w = 1 / target_ratio

    # preprocessing
    x = imgproc.normalizeMeanVariance(img_resized)
    x = torch.from_numpy(x).permute(2, 0, 1)  # [h, w, c] to [c, h, w]
    x = Variable(x.unsqueeze(0))  # [c, h, w] to [b, c, h, w]
    if cuda:
        x = x.cuda()

    # forward pass
    with torch.no_grad():
        y, feature = net(x)

    # make score and link map
    score_text = y[0, :, :, 0].cpu().data.numpy()
    score_link = y[0, :, :, 1].cpu().data.numpy()

    # refine link
    if refine_net is not None:
        with torch.no_grad():
            y_refiner = refine_net(y, feature)
        score_link = y_refiner[0, :, :, 0].cpu().data.numpy()

    t0 = time.time() - t0
    t1 = time.time()

    # Post-processing
    boxes, polys = craft_utils.getDetBoxes(
        score_text, score_link, text_threshold, link_threshold, low_text, poly
    )

    # coordinate adjustment
    boxes = craft_utils.adjustResultCoordinates(boxes, ratio_w, ratio_h)
    polys = craft_utils.adjustResultCoordinates(polys, ratio_w, ratio_h)
    for k in range(len(polys)):
        if polys[k] is None:
            polys[k] = boxes[k]

    t1 = time.time() - t1

    # render results (optional)
    render_img = score_text.copy()
    render_img = np.hstack((render_img, score_link))
    ret_score_text = imgproc.cvt2HeatmapImg(render_img)

    return boxes, polys, ret_score_text


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
    lines = string.split('\n')
    timestamps = re.findall(r"[-+]?\d*\.\d+|\d+", lines[0])
    vector = re.findall(r"[-+]?\d*\.\d+|\d+", lines[1])
    distance = re.findall(r"[-+]?\d*\.\d+|\d+", lines[2])
    pcl = re.findall(r"[-+]?\d*\.\d+|\d+", lines[3])
    pcl = [float(i) for i in pcl]
    mesh = re.findall(r"[-+]?\d*\.\d+|\d+", lines[4])
    mesh = [float(i) for i in mesh]
    results = {'Closest Timestamp Depth': float(timestamps[0]), 
                'Closest Timestamp Pose': float(timestamps[1]),
                'Vector': tuple(float(i) for i in vector),
                'Point Cloud to Vector Distance': float(distance[0]),
                'PCL Intersection': None,
                'Mesh Intersection': None}

    if len(pcl) != 0:
        results['PCL Intersection'] = {'x': pcl[0], 'y': pcl[1], 'z': pcl[2], 'roll': pcl[3], 'pitch': pcl[4], 'yaw': pcl[5]}
    
    if len(mesh) != 0:
        results['Mesh Intersection'] = {'x': mesh[0], 'y': mesh[1], 'z': mesh[2], 'roll': mesh[3], 'pitch': mesh[4], 'yaw': mesh[5]}

    return results

def decode_image(
    image_path, result_folder=None, trained_model="models/craft_mlt_25k.pth"
):
    """
    @param image_path path to image to parse
    @param result_folder if provided will save the marked image with boundary boxes and labels to the folder
    @param trained_model path to trained model
    @returns database of labels database['label'] and boundary boxes database['location']
             result_image array representing labeled image
             image array representing original nonlabeled image
    """
    cuda = False
    refine = False
    poly = False
    # show_time = False
    text_threshold = 0.7
    low_text = 0.4
    link_threshold = 0.1
    # mag_ratio = 1.5

    refiner_model = "weights/craft_refiner_CTW1500.pth"
    # trained_model = 'models/craft_mlt_25k.pth'

    if result_folder is not None and not os.path.isdir(result_folder):
        os.mkdir(result_folder)
    # ============================ Initialization ============================

    filename, file_ext = os.path.splitext(os.path.basename(image_path))

    bag_name = get_bag_file(float(filename))
    if bag_name is not None:
        bag_name = '/srv/novus_1/mgouveia/data/bags/20220711_Isaac11/queen/' + bag_name
    else:
        return None

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

    # load net
    net = CRAFT()  # initialize

    print("Loading weights from checkpoint (" + trained_model + ")")
    if cuda:
        net.load_state_dict(copyStateDict(torch.load(trained_model)))
    else:
        net.load_state_dict(
            copyStateDict(torch.load(trained_model, map_location="cpu"))
        )

    if cuda:
        net = net.cuda()
        net = torch.nn.DataParallel(net)
        cudnn.benchmark = False

    net.eval()
    refine_net = None
    result_path = None

    # Load model and image transforms for parseq
    parseq = torch.hub.load("baudm/parseq", "parseq", pretrained=True).eval()
    img_transform = SceneTextDataModule.get_transform(parseq.hparams.img_size)

    df = pd.DataFrame(columns=["label", "location"])
    # ============================ Start Processing ============================

    t = time.time()

    # load data
    image = cv2.imread(image_path)

    h, w, _ = image.shape

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

    bboxes, polys, score_text = test_net(
        net,
        image,
        text_threshold,
        link_threshold,
        low_text,
        cuda,
        poly,
        refine_net,
    )

    # print(polys)
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

        # Find 3D position
        # execute_command = ['./executable', 'param1', 'param2', 'param3']
        # subprocess.run(execute_command, check=True)

        if overlap_result.empty:
            # print('empty')
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
            print("\rTest part {:d}/{:d}".format(num + 1, total))
            bboxes, polys, score_text = test_net(
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
                        # print(label[0], row['label'])
                        old_label = row["label"]
                        old_location = row["location"]
                        if similar(old_label, label[0]):
                            # print('similar')
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

    result_image = display_all(image, df, result_path)
    locations = get_all_locations(df, data, bag_name, ros_command, result_path)
    print("elapsed time : {}s".format(time.time() - t))
    return df, result_image, image, locations

def get_all_locations(database, data, bag_name, ros_command, result_path):
    locations = {}
    total = len(database)
    with open(result_path[:-4] + '_locations.txt', 'w') as f:
        for i, row in database.iterrows():
            print("\rGetting Locations {:d}/{:d}".format(i+1, total))
            label = row['label']
            new_location = row['location']
            data["coord"]["x"] = int(
                (new_location[1][0] + new_location[0][0]) / 2
            )
            data["coord"]["y"] = int(
                (new_location[1][1] + new_location[0][1]) / 2
            )

            json_file = ros_command[-1]

            with open(json_file, "w") as file:
                json.dump(data, file)

            # Run the ROS command using subprocess
            process = subprocess.Popen(
                ros_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )

            # Wait for the process to finish and capture the output
            stdout, stderr = process.communicate()
            print('error: ', stderr)
            stdout = stdout.decode()
            stderr = stderr.decode()
            if len(stderr) != 0:
                continue
            
            result_positions = parse_3D_result(stdout)

            locations[label] = result_positions['PCL Intersection']
            # print(result_positions)
            f.write('Label:%s\n' % label)
            for key, value in result_positions.items(): 
                f.write('%s:%s\n' % (key, value))
            f.write('\n')
    return locations

def get_closest_rect(rect, rectangles, distance):
    rects = []
    for r in rectangles:
        d = utils.get_rect_distance(rect[0], rect[1], r[0], r[1])
        if d < distance:
            rects.append(r)
    return rects


def display_all(image, database, result_path=None):
    """
    @param image    openCV array of image
    @param database pandas database of columns 'label' and 'location' where 'location' is the upper left
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


def find(image, database, label):
    """
    @param image array representing image
    @param database database of labels (database['labels']) and their corresponding boundary boxes (database['location'])
    @param label text to search for
    @returns array representing image with label boxed if found and a list of all labels cropped from original image
    """
    words = label.split()
    results = {}
    for l in words:
        l_result = database.loc[database["label"].apply(similar, args=(l,))]
        results[l] = l_result["location"].tolist()

    rectangles = np.array(results[words[0]])
    for word in words[1:]:
        new_rects = []
        other_rects = results[word]
        for rect in rectangles:
            # letter_width = round((rect[1][0] - rect[0][0])/len(word))
            limit = 2 * (rect[1][1] - rect[0][1])
            # print(rect, other_rects)
            closest_rect = get_closest_rect(rect, other_rects, limit)
            for r in closest_rect:
                new_rects.append(utils.get_bounding_box(rect, r))
        rectangles = new_rects
    new_image = np.array(image)

    offset = 10
    cropped_images = []
    for rect in rectangles:
        new_image = cv2.rectangle(new_image, rect[0], rect[1], (255, 0, 0), 10)
        cropped_images.append(utils.crop_image(image, rect[0][0] - offset, rect[0][1] - offset, rect[1][0] + offset, rect[1][1] + offset))

    return new_image, cropped_images

if __name__ == "__main__":
    warnings.filterwarnings("ignore")
    os.environ[
        "ASTROBEE_CONFIG_DIR"
    ] = "/home/rlu3/astrobee/src/astrobee/config"
    os.environ[
        "ASTROBEE_RESOURCE_DIR"
    ] = "/home/rlu3/astrobee/src/astrobee/resource"
    os.environ["ASTROBEE_ROBOT"] = "queen"
    os.environ["ASTROBEE_WORLD"] = "iss"

    test_image = "/srv/novus_1/mgouveia/data/bags/20220711_Isaac11/queen/isaac_sci_cam_image_delayed/1657544476.435.jpg"
    result_folder = "result/beehive/queen/"

    test_folder = "/srv/novus_1/mgouveia/data/bags/20220711_Isaac11/queen/isaac_sci_cam_image_delayed/"
    image_list, _, _ = file_utils.get_files(test_folder)

    for k, image_path in enumerate(image_list):
        print(
            "\rTest image {:d}/{:d}: {:s}".format(k + 1, len(image_list), image_path)
        )
        result = decode_image(image_path, result_folder)
        if result is None:
            print('Skipped Image')
            continue
        database, result_image, image, locations = result
        # print(database)
    # result = decode_image(test_image, result_folder)
    # database, result_image, image, locations = result
    # print(database)

    IPython.embed()
