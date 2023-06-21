import sys

from parseq.strhub.data.module import SceneTextDataModule

from PIL import Image

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
import craft.craft_utils as craft_utils
import craft.imgproc as imgproc
import craft.file_utils as file_utils
import json
import zipfile

from craft.craft import CRAFT
from collections import OrderedDict
import pandas as pd
import IPython
import jellyfish

# def get_rect_distance(lower_a, upper_a, lower_b, upper_b):
#     delta1 = lower_a - upper_b
#     delta2 = lower_b - upper_a
#     u = np.max(np.array([np.zeros(len(delta1)), delta1]), axis=0)
#     v = np.max(np.array([np.zeros(len(delta2)), delta2]), axis=0)
#     dist = np.linalg.norm(np.concatenate([u, v]))
#     return dist

def crop_image(img, startx, starty, endx, endy):
    """
    @param img array of image
    @param startx
    @param starty
    @param endx 
    @param endy
    @returns a cropped image of img[startx:endx, starty:endy] 
    """
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

def overlap(rect1, rect2):
    # rect1 and rect2 are tuples in the form ((x1, y1), (x2, y2))
    # representing the upper left and lower right points of each rectangle

    upper1, lower1 = rect1
    upper2, lower2 = rect2
    
    return lower1[0] >= upper2[0] and upper1[0] <= lower2[0] and lower1[1] >= upper2[1] and upper1[1] <= lower2[1]  

def get_bounding_box(rect1, rect2):
    # rect1 and rect2 are tuples in the form ((x1, y1), (x2, y2))
    # representing the upper left and lower right points of each rectangle

    upper_left = (min(rect1[0][0], rect2[0][0]), min(rect1[0][1], rect2[0][1]))
    lower_right = (max(rect1[1][0], rect2[1][0]), max(rect1[1][1], rect2[1][1]))

    return (upper_left, lower_right)

    
def decode_image(image_path, result_folder):
    cuda = False
    refine = False
    poly = False
    show_time = False

    text_threshold = 0.7
    low_text = 0.4
    link_threshold = 0.4
    mag_ratio = 1.5

    refiner_model = 'weights/craft_refiner_CTW1500.pth'
    trained_model = 'models/craft_mlt_25k.pth'

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

    df = pd.DataFrame(columns=['label', 'location'])
    # ============================ Start Processing ============================

    t = time.time()

    filename, file_ext = os.path.splitext(os.path.basename(image_path))
    result_path = result_folder + filename + '/'
    if not os.path.isdir(result_folder):
            os.mkdir(result_folder)

    # load data
    image = cv2.imread(image_path)
    
    h, w, _ = image.shape
    # print(w, h, '\n')

    crop_w = 1200
    crop_h = 1200
    # print(crop_w, crop_h, '\n')
    offset_w = 800
    offset_h = 800
    # print(offset_w, offset_h, '\n')

    end_w = max(w-crop_w+offset_w, offset_w)
    end_h = max(h-crop_h+offset_h, offset_h)

    edge_border = 15
    total = round((end_w/offset_w+1) * (end_h/offset_h+1))
    num = 0
    boundary = 10

    for x in range(0, end_w, offset_w):
        for y in range(0, end_h, offset_h):
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
                    # Box in form upper left -> upper right -> lower right -> lower left, (x, y)
                    coordinates = [int(num) for num in box.split(',')]
                    x_coordinates = coordinates[::2]
                    y_coordinates = coordinates[1::2]
                    upper_left = (min(x_coordinates), min(y_coordinates))
                    lower_right = (max(x_coordinates), max(y_coordinates))
                    cropped_image = crop_image(img, upper_left[0], upper_left[1], lower_right[0], lower_right[1])
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

                    # Convert to location on original image
                    upper_left = (upper_left[0] + x, upper_left[1] + y)
                    lower_right = (lower_right[0] + x, lower_right[1] + y)
                    new_location = (upper_left, lower_right)
                    overlap_result = df.loc[df['location'].apply(overlap, args=(new_location,  ))]

                    if overlap_result.empty:
                        df.loc[len(df)] = [label[0], (upper_left, lower_right)]
                    else:
                        index = overlap_result.index[0]
                        old_label = df.at[index, 'label']
                        old_location = df.at[index, 'location']
                        new_label = old_label if len(old_label) >= len(label[0]) else label[0]
                        new_location = get_bounding_box(old_location, new_location)
                        new_row = np.array([new_label, new_location], dtype=object)
                        df.iloc[index] = new_row

                    # print('Decoded label = {}\n'.format(label[0]))

                    # cv2.imshow('image', cropped_image)
                    # cv2.waitKey(0)
    print("elapsed time : {}s".format(time.time() - t))
    return df, image

def find(image, database, label):
    def compare(label, input_label):

        return jellyfish.jaro_distance(label.upper(), input_label.upper()) > 0.8
    result = database.loc[database['label'].apply(compare, args=(label, ))]
    print(result)
    rectangles = result['location'].tolist()
    new_image = image
    for rect in rectangles:
        new_image = cv2.rectangle(new_image, rect[0], rect[1], (255, 0, 0), 10)
    cv2.namedWindow("Image", 0)
    cv2.resizeWindow('Image', 1000, 1000)
    cv2.imshow('Image', new_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    test_image = 'images/ISS.jpg'
    result_folder = 'result/craft_parseq/'

    database, image = decode_image(test_image, result_folder)
    print(database)    

    IPython.embed()

