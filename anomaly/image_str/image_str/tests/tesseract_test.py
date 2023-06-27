import sys

sys.path.insert(1, "/home/astrobee/ros_ws/isaac/src/anomaly/str/parseq")
sys.path.insert(2, "/home/astrobee/ros_ws/isaac/src/anomaly/str/craft")
import json
import os
import time
import zipfile
from collections import OrderedDict

import craft_utils
import cv2
import file_utils
import imgproc
import numpy as np
import pytesseract
import torch
import torch.backends.cudnn as cudnn
import torch.nn as nn
from craft import CRAFT
from PIL import Image
from skimage import io
from strhub.data.module import SceneTextDataModule
from torch.autograd import Variable


def crop_image(img, startx, starty, endx, endy):
    h, w, _ = image.shape

    startx = max(0, startx)
    starty = max(0, starty)

    endx = min(endx, w)
    endy = min(endy, h)

    return img[starty:endy, startx:endx]


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


def str2bool(v):
    return v.lower() in ("yes", "y", "true", "t", "1")


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


cuda = False
refine = False
poly = False
show_time = False

text_threshold = 0.7
low_text = 0.4
link_threshold = 0.4
mag_ratio = 1.5

refiner_model = "weights/craft_refiner_CTW1500.pth"
trained_model = "../models/craft_mlt_25k.pth"

test_folder = "../images/"
result_folder = "../result/trade_study/tesseract/"

image_list, _, _ = file_utils.get_files(test_folder)
if not os.path.isdir(result_folder):
    os.mkdir(result_folder)


if __name__ == "__main__":
    # =================== CRAFT ===================
    print("=================== CRAFT ===================")

    # load net
    net = CRAFT()  # initialize

    # Load model and image transforms for parseq
    parseq = torch.hub.load("baudm/parseq", "parseq", pretrained=True).eval()
    img_transform = SceneTextDataModule.get_transform(parseq.hparams.img_size)

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

    # LinkRefiner
    refine_net = None
    if refine:
        from refinenet import RefineNet

        refine_net = RefineNet()
        print("Loading weights of refiner from checkpoint (" + refiner_model + ")")
        if cuda:
            refine_net.load_state_dict(copyStateDict(torch.load(refiner_model)))
            refine_net = refine_net.cuda()
            refine_net = torch.nn.DataParallel(refine_net)
        else:
            refine_net.load_state_dict(
                copyStateDict(torch.load(refiner_model, map_location="cpu"))
            )

        refine_net.eval()
        poly = True

    t = time.time()

    # load data
    for k, image_path in enumerate(image_list):
        print(
            "Test image {:d}/{:d}: {:s}\n".format(k + 1, len(image_list), image_path),
            end="\r",
        )
        image = imgproc.loadImage(image_path)

        bboxes, polys, score_text = test_net(
            net, image, text_threshold, link_threshold, low_text, cuda, poly, refine_net
        )

        # save score text
        filename, file_ext = os.path.splitext(os.path.basename(image_path))
        mask_file = result_folder + "/res_" + filename + "_mask.jpg"
        cv2.imwrite(mask_file, score_text)

        file_utils.saveResult(
            image_path, image[:, :, ::-1], polys, dirname=result_folder
        )

        # ============== parseq ============== #
        boxes = open(result_folder + "res_" + filename + ".txt", "r")
        lines = boxes.readlines()
        i = 1

        image_result_folder = result_folder + filename + "/"
        if not os.path.isdir(image_result_folder):
            os.mkdir(image_result_folder)

        for box in lines:
            print(box)
            coordinates = [int(num) for num in box.split(",")]
            x_coordinates = coordinates[::2]
            y_coordinates = coordinates[1::2]
            cropped_image = crop_image(
                image,
                min(x_coordinates),
                min(y_coordinates),
                max(x_coordinates),
                max(y_coordinates),
            )

            cv2.imwrite(image_result_folder + str(i) + ".jpg", cropped_image)
            new_img = Image.fromarray(np.array(cropped_image)).convert("RGB")

            print(pytesseract.image_to_string(image_result_folder + str(i) + ".jpg"))

            cv2.imshow("image", cropped_image)
            cv2.waitKey(0)
            i += 1

    print("elapsed time : {}s".format(time.time() - t))
