import sys
sys.path.insert(1, '/home/astrobee/ros_ws/isaac/src/anomaly/str/parseq')
sys.path.insert(2, '/home/astrobee/ros_ws/isaac/src/anomaly/str/craft')

from PIL import Image

from strhub.data.module import SceneTextDataModule

import os
import time

import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
from torch.autograd import Variable

from PIL import Image

from skimage import io

import cv2
import numpy as np
import craft_utils
import imgproc
import file_utils
import json
import zipfile

from craft import CRAFT
from collections import OrderedDict

# def get_rect_distance(lower_a, upper_a, lower_b, upper_b):
#     delta1 = lower_a - upper_b
#     delta2 = lower_b - upper_a
#     u = np.max(np.array([np.zeros(len(delta1)), delta1]), axis=0)
#     v = np.max(np.array([np.zeros(len(delta2)), delta2]), axis=0)
#     dist = np.linalg.norm(np.concatenate([u, v]))
#     return dist

def crop_image(img, startx, starty, endx, endy):
    h, w, _ = img.shape

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

def test_net(net, image, text_threshold, link_threshold, low_text, cuda, poly, refine_net=None):
    t0 = time.time()

    canvas_size = 1280
    mag_ratio = 1.5

    # resize
    img_resized, target_ratio, size_heatmap = imgproc.resize_aspect_ratio(image, canvas_size, interpolation=cv2.INTER_LINEAR, mag_ratio=mag_ratio)
    ratio_h = ratio_w = 1 / target_ratio

    # preprocessing
    x = imgproc.normalizeMeanVariance(img_resized)
    x = torch.from_numpy(x).permute(2, 0, 1)    # [h, w, c] to [c, h, w]
    x = Variable(x.unsqueeze(0))                # [c, h, w] to [b, c, h, w]
    if cuda:
        x = x.cuda()

    # forward pass
    with torch.no_grad():
        y, feature = net(x)

    # make score and link map
    score_text = y[0,:,:,0].cpu().data.numpy()
    score_link = y[0,:,:,1].cpu().data.numpy()

    # refine link
    if refine_net is not None:
        with torch.no_grad():
            y_refiner = refine_net(y, feature)
        score_link = y_refiner[0,:,:,0].cpu().data.numpy()

    t0 = time.time() - t0
    t1 = time.time()

    # Post-processing
    boxes, polys = craft_utils.getDetBoxes(score_text, score_link, text_threshold, link_threshold, low_text, poly)

    # coordinate adjustment
    boxes = craft_utils.adjustResultCoordinates(boxes, ratio_w, ratio_h)
    polys = craft_utils.adjustResultCoordinates(polys, ratio_w, ratio_h)
    for k in range(len(polys)):
        if polys[k] is None: polys[k] = boxes[k]

    t1 = time.time() - t1

    # render results (optional)
    render_img = score_text.copy()
    render_img = np.hstack((render_img, score_link))
    ret_score_text = imgproc.cvt2HeatmapImg(render_img)

    return boxes, polys, ret_score_text

def decode_img(image_folder, result_folder):
    cuda = False
    refine = False
    poly = False
    show_time = False

    text_threshold = 0.7
    low_text = 0.4
    link_threshold = 0.4
    mag_ratio = 1.5

    refiner_model = 'weights/craft_refiner_CTW1500.pth'
    trained_model = '../models/craft_mlt_25k.pth'

    if not os.path.isdir(result_folder):
        os.mkdir(result_folder)
    # ============================ Initialization ============================

    # load net
    net = CRAFT()     # initialize

    print('Loading weights from checkpoint (' + trained_model + ')')
    if cuda:
        net.load_state_dict(copyStateDict(torch.load(trained_model)))
    else:
        net.load_state_dict(copyStateDict(torch.load(trained_model, map_location='cpu')))

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
        print('Loading weights of refiner from checkpoint (' + refiner_model + ')')
        if cuda:
            refine_net.load_state_dict(copyStateDict(torch.load(refiner_model)))
            refine_net = refine_net.cuda()
            refine_net = torch.nn.DataParallel(refine_net)
        else:
            refine_net.load_state_dict(copyStateDict(torch.load(refiner_model, map_location='cpu')))

        refine_net.eval()
        poly = True

    # Load model and image transforms for parseq
    parseq = torch.hub.load('baudm/parseq', 'parseq', pretrained=True).eval()
    img_transform = SceneTextDataModule.get_transform(parseq.hparams.img_size)

    # ============================ Start Processing ============================

    t = time.time()
    image_list, _, _ = file_utils.get_files(test_folder)

    for k, image_path in enumerate(image_list):
        filename, file_ext = os.path.splitext(os.path.basename(image_path))
        result_path = result_folder + filename + '/'
        if not os.path.isdir(result_folder):
                os.mkdir(result_folder)

        print("Test image {:d}/{:d}: {:s} {:s}\n".format(k+1, len(image_list), image_path, result_path), end='\r')
        # load data
        image = cv2.imread(image_path)
        
        h, w, _ = image.shape
        # print(w, h, '\n')

        crop_w = 1200
        crop_h = 1200
        # print(crop_w, crop_h, '\n')
        offset_w = 500
        offset_h = 500
        # print(offset_w, offset_h, '\n')

        edge_border = 15

        total = int((w-crop_w+offset_w)/offset_w * (h-crop_h+offset_h)/offset_h)
        num = 0
        boundary = 10
        for x in range(0, w-crop_w+offset_w, offset_w):
            for y in range(0, h-crop_h+offset_h, offset_h):
                img = crop_image(image, x, y, x+crop_w, y+crop_h)
                print("Test part {:d}/{:d}".format(num+1, total), end='\r')
                bboxes, polys, score_text = test_net(net, img, text_threshold, link_threshold, low_text, cuda, poly, refine_net)

                # save score text
                mask_file = result_path + "res_" + str(num) + '_mask.jpg'
                cv2.imwrite(mask_file, score_text)

                file_utils.saveResult(str(num), img[:,:,::-1], polys, dirname=result_path)
                num += 1

                # # ============== parseq ============== #
                boxes = open(result_path + 'res_' + str(num-1) + '.txt', 'r')
                lines = boxes.readlines()
                decode_file = result_path + 'decode_' + str(num-1) + '.txt'
                with open(decode_file, 'w') as f:
                    for box in lines:
                        coordinates = [int(num) for num in box.split(',')]
                        x_coordinates = coordinates[::2]
                        y_coordinates = coordinates[1::2]
                        cropped_image = crop_image(img, min(x_coordinates), min(y_coordinates), max(x_coordinates), max(y_coordinates))
                        if (min(x_coordinates) < edge_border or max(x_coordinates) > crop_w-edge_border
                                or min(y_coordinates) < edge_border or max(y_coordinates) > crop_h-edge_border):
                            continue
                        new_img = Image.fromarray(np.array(cropped_image)).convert('RGB')
                        new_img = img_transform(new_img).unsqueeze(0)

                        logits = parseq(new_img)
                        logits.shape  # torch.Size([1, 26, 95]), 94 characters + [EOS] symbol

                        # Greedy decoding
                        pred = logits.softmax(-1)
                        label, confidence = parseq.tokenizer.decode(pred)

                        strResult = box + ' ' + label[0] + '\r\n'
                        f.write(strResult)

                        # print('Decoded label = {}\n'.format(label[0]))

                        # cv2.imshow('image', cropped_image)
                        # cv2.waitKey(0)

        print("elapsed time : {}s".format(time.time() - t))

if __name__ == '__main__':
    test_folder = '../images/'
    result_folder = '../result/craft_parseq/'

    decode_img(test_folder, result_folder)