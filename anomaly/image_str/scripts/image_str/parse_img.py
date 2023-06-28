import os
import time
from collections import OrderedDict

import cv2
import craft.craft_utils as craft_utils
import craft.file_utils as file_utils
import craft.imgproc as imgproc
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

def decode_image(image_path, result_folder, write=True, trained_model='models/craft_mlt_25k.pth'):
    '''
    @param image_path
    @param result_folder
    @param write
    @param trained_model
    @
    '''
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

    if not os.path.isdir(result_folder):
        os.mkdir(result_folder)
    # ============================ Initialization ============================

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

    # Load model and image transforms for parseq
    parseq = torch.hub.load("baudm/parseq", "parseq", pretrained=True).eval()
    img_transform = SceneTextDataModule.get_transform(parseq.hparams.img_size)

    df = pd.DataFrame(columns=["label", "location"])
    # ============================ Start Processing ============================

    t = time.time()

    filename, file_ext = os.path.splitext(os.path.basename(image_path))
    result_path = result_folder + filename + '.jpg'

    # load data
    image = cv2.imread(image_path)

    h, w, _ = image.shape

    crop_w = 1500
    crop_h = 1500
    offset_w = 1000
    offset_h = 1000

    end_w = max(w - crop_w + offset_w, offset_w)
    end_h = max(h - crop_h + offset_h, offset_h)

    edge_border = 15
    total = round((end_w / offset_w) * (end_h / offset_h))
    num = 0

    for x in range(0, end_w, offset_w):
        for y in range(0, end_h, offset_h):
            img = utils.crop_image(image, x, y, x+crop_w, y+crop_h)
            print("Test part {:d}/{:d}".format(num+1, total), end='\r')
            bboxes, polys, score_text = test_net(net, img, text_threshold, link_threshold, low_text, cuda, poly, refine_net)
            # save score text
            # mask_file = result_path + "res_" + str(num) + '_mask.jpg'
            # cv2.imwrite(mask_file, score_text)

            # file_utils.saveResult(str(num), img[:,:,::-1], polys, dirname=result_path)
            num += 1

            # # ============== parseq ============== #
            # boxes = open(result_path + 'res_' + str(num-1) + '.txt', 'r')
            # lines = boxes.readlines()
            # decode_file = result_path + 'decode_' + str(num-1) + '.txt'
            # with open(decode_file, 'w') as f:
            for box in polys:
                # Box in form upper left -> upper right -> lower right -> lower left, (x, y)
                box = np.array(box).astype(np.int32)
                x_coordinates = box[:, 0]
                y_coordinates = box[:, 1]
                upper_left = np.array((min(x_coordinates), min(y_coordinates)))
                lower_right = np.array((max(x_coordinates), max(y_coordinates)))
                cropped_image = utils.crop_image(img, upper_left[0], upper_left[1], lower_right[0], lower_right[1])
                if (max(x_coordinates) < edge_border or min(x_coordinates) > crop_w-edge_border
                    or max(y_coordinates) < edge_border or min(y_coordinates) > crop_h-edge_border):
                    continue
                new_img = Image.fromarray(np.array(cropped_image)).convert('RGB')
                new_img = img_transform(new_img).unsqueeze(0)

                logits = parseq(new_img)
                logits.shape  # torch.Size([1, 26, 95]), 94 characters + [EOS] symbol

                # Greedy decoding
                pred = logits.softmax(-1)
                label, confidence = parseq.tokenizer.decode(pred)

                # strResult = box + ' ' + label[0] + '\r\n'
                # f.write(strResult)

                # Convert to location on original image
                upper_left = np.array((upper_left[0] + x, upper_left[1] + y))
                lower_right = np.array((lower_right[0] + x, lower_right[1] + y))
                new_location = np.array((upper_left, lower_right))
                overlap_result = df.loc[df['location'].apply(utils.overlap, args=(new_location,  ))]

                if overlap_result.empty:
                    df.loc[len(df)] = [label[0], np.array((upper_left, lower_right))]
                else:
                    for index, row in overlap_result.iterrows():
                        old_label = row['label']
                        old_location = row['location']
                        if similar(old_label, label[0]):
                            new_label = old_label if len(old_label) >= len(label[0]) else label[0]
                            new_location = utils.get_bounding_box(old_location, new_location)
                            new_row = np.array([new_label, new_location], dtype=object)
                            df.iloc[index] = new_row

    image = display_all(image, df, result_path, write)
    print("elapsed time : {}s".format(time.time() - t))
    return df, image


def get_closest_rect(rect, rectangles, distance):
    rects = []
    for r in rectangles:
        d = utils.get_rect_distance(rect[0], rect[1], r[0], r[1])
        if d < distance:
            rects.append(r)
    return rects

def display_all(image, database, result_path, write=True):
    '''
    @param image    openCV array of image
    @param database pandas database of columns 'label' and 'location' where 'location' is the upper left 
                    and lower right points of the rectangular bounding box for the corresponding label
    @returns array of image with the labels and boundary boxes displayed
    '''
    display_image = image.copy()
    blue = (255, 0, 0)
    red = (0, 0, 255)
    thickness = 2
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    for _, row in database.iterrows():
        rect = row['location']
        display_image = cv2.rectangle(display_image, rect[0], rect[1], blue, 10)
        display_image = cv2.putText(display_image, row['label'], rect[0], font, 1, red, thickness, cv2.LINE_AA)

    if write:
        cv2.imwrite(result_path, display_image)

    return display_image

def similar(label, input_label):
    # [0, 1] where 0 represents two completely dissimilar strings and 1 represents identical strings
    label = label.upper()
    input_label = input_label.upper()
    return jellyfish.jaro_winkler_similarity(label, input_label) > 0.8


def find(image, database, label):
    words = label.split()
    results = {}
    for l in words:
        l_result = database.loc[database['label'].apply(similar, args=(l, ))]
        results[l] = l_result['location'].tolist()

    rectangles = np.array(results[words[0]])
    print(results)
    for word in words[1:]:
        new_rects = []
        other_rects = results[word]
        for rect in rectangles:
            # letter_width = round((rect[1][0] - rect[0][0])/len(word))
            limit = 2*(rect[1][1] - rect[0][1])
            # print(rect, other_rects)
            closest_rect = get_closest_rect(rect, other_rects, limit)
            for r in closest_rect:
                new_rects.append(utils.get_bounding_box(rect, r))
        rectangles = new_rects

    # result = database.loc[database['label'].apply(compare, args=(label, ))]
    # rectangles = result['location'].tolist()
    new_image = np.array(image)
    for rect in rectangles:
        new_image = cv2.rectangle(new_image, rect[0], rect[1], (255, 0, 0), 10)
    cv2.namedWindow("Image", 0)
    cv2.resizeWindow("Image", 1000, 1000)
    cv2.imshow("Image", new_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return new_image

if __name__ == '__main__':
    test_image = 'images/ISS.jpg'
    result_folder = 'result/final/'

    test_folder = 'images/'
    image_list, _, _ = file_utils.get_files(test_folder)

    # for k, image_path in enumerate(image_list):
    #     print("Test image {:d}/{:d}: {:s}\n".format(k+1, len(image_list), image_path), end='\r')
    #     database, image = decode_image(image_path, result_folder)
    #     print(database)

    database, image = decode_image(test_image, result_folder)
    print(database)

    IPython.embed()
