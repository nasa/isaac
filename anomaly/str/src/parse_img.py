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

import cv2
from skimage import io
import numpy as np
import craft_utils
import imgproc
import file_utils
import json
import zipfile

from craft import CRAFT
from collections import OrderedDict

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

test_image = '../images/ISS.jpg'
result_folder = '../result/'

if not os.path.isdir(result_folder):
    os.mkdir(result_folder)


if __name__ == '__main__':
    # =================== CRAFT ===================
    print('=================== CRAFT ===================')

    # load net
    net = CRAFT()     # initialize

    # Load model and image transforms for parseq
    parseq = torch.hub.load('baudm/parseq', 'parseq', pretrained=True).eval()
    img_transform = SceneTextDataModule.get_transform(parseq.hparams.img_size)

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

    t = time.time()

    # load data
    image = cv2.imread(test_image)
    
    h, w, _ = image.shape
    half = w//2
    half2 = h//2

    image1 = image[:half2, :half]
    image2 = image[half2:, :half]
    image3 = image[half2:, half:]
    image4 = image[:half2, half:]

    for num, img in enumerate([image1, image2, image3, image4]):
        print("Test image {:d}/{:d}: {:s}".format(num+1, 4, test_image), end='\n')
        bboxes, polys, score_text = test_net(net, img, text_threshold, link_threshold, low_text, cuda, poly, refine_net)

        # save score text
        mask_file = result_folder + "/res_" + str(num) + '_mask.jpg'
        cv2.imwrite(mask_file, score_text)

        file_utils.saveResult(str(num), img[:,:,::-1], polys, dirname=result_folder)

    # ============== parseq ==============
    img = Image.open(test_image).convert('RGB')
    # Preprocess. Model expects a batch of images with shape: (B, C, H, W)
    img = img_transform(img).unsqueeze(0)

    logits = parseq(img)
    logits.shape  # torch.Size([1, 26, 95]), 94 characters + [EOS] symbol

    # Greedy decoding
    pred = logits.softmax(-1)
    label, confidence = parseq.tokenizer.decode(pred)
    print('Decoded label = {}\n'.format(label[0]))

    print("elapsed time : {}s".format(time.time() - t))
