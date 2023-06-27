#!/usr/bin/env python3
# Copyright (c) OpenMMLab. All rights reserved.
import argparse
import math
import os
import os.path as osp
from functools import partial

import mmcv
import numpy as np
from PIL import Image
from mmocr.utils.fileio import list_to_file


def parse_args():
    parser = argparse.ArgumentParser(
        description='Generate training and validation set of TextOCR '
                    'by cropping box image.')
    parser.add_argument('root_path', help='Root dir path of TextOCR')
    parser.add_argument(
        'n_proc', default=1, type=int, help='Number of processes to run')
    parser.add_argument('--rectify_pose', action='store_true',
                        help='Fix pose of rotated text to make them horizontal')
    args = parser.parse_args()
    return args


def rectify_image_pose(image, top_left, points):
    # Points-based heuristics for determining text orientation w.r.t. bounding box
    points = np.asarray(points).reshape(-1, 2)
    dist = ((points - np.asarray(top_left)) ** 2).sum(axis=1)
    left_midpoint = (points[0] + points[-1]) / 2
    right_corner_points = ((points - left_midpoint) ** 2).sum(axis=1).argsort()[-2:]
    right_midpoint = points[right_corner_points].sum(axis=0) / 2
    d_x, d_y = abs(right_midpoint - left_midpoint)

    if dist[0] + dist[-1] <= dist[right_corner_points].sum():
        if d_x >= d_y:
            rot = 0
        else:
            rot = 90
    else:
        if d_x >= d_y:
            rot = 180
        else:
            rot = -90
    if rot:
        image = image.rotate(rot, expand=True)
    return image


def process_img(args, src_image_root, dst_image_root):
    # Dirty hack for multiprocessing
    img_idx, img_info, anns, rectify_pose = args
    src_img = Image.open(osp.join(src_image_root, img_info['file_name']))
    labels = []
    for ann_idx, ann in enumerate(anns):
        text_label = ann['utf8_string']

        # Ignore illegible or non-English words
        if text_label == '.':
            continue

        x, y, w, h = ann['bbox']
        x, y = max(0, math.floor(x)), max(0, math.floor(y))
        w, h = math.ceil(w), math.ceil(h)
        dst_img = src_img.crop((x, y, x + w, y + h))
        if rectify_pose:
            dst_img = rectify_image_pose(dst_img, (x, y), ann['points'])
        dst_img_name = f'img_{img_idx}_{ann_idx}.jpg'
        dst_img_path = osp.join(dst_image_root, dst_img_name)
        # Preserve JPEG quality
        dst_img.save(dst_img_path, qtables=src_img.quantization)
        labels.append(f'{osp.basename(dst_image_root)}/{dst_img_name}'
                      f' {text_label}')
    src_img.close()
    return labels


def convert_textocr(root_path,
                    dst_image_path,
                    dst_label_filename,
                    annotation_filename,
                    img_start_idx=0,
                    nproc=1,
                    rectify_pose=False):
    annotation_path = osp.join(root_path, annotation_filename)
    if not osp.exists(annotation_path):
        raise Exception(
            f'{annotation_path} not exists, please check and try again.')
    src_image_root = root_path

    # outputs
    dst_label_file = osp.join(root_path, dst_label_filename)
    dst_image_root = osp.join(root_path, dst_image_path)
    os.makedirs(dst_image_root, exist_ok=True)

    annotation = mmcv.load(annotation_path)

    process_img_with_path = partial(
        process_img,
        src_image_root=src_image_root,
        dst_image_root=dst_image_root)
    tasks = []
    for img_idx, img_info in enumerate(annotation['imgs'].values()):
        ann_ids = annotation['imgToAnns'][img_info['id']]
        anns = [annotation['anns'][ann_id] for ann_id in ann_ids]
        tasks.append((img_idx + img_start_idx, img_info, anns, rectify_pose))
    labels_list = mmcv.track_parallel_progress(
        process_img_with_path, tasks, keep_order=True, nproc=nproc)
    final_labels = []
    for label_list in labels_list:
        final_labels += label_list
    list_to_file(dst_label_file, final_labels)
    return len(annotation['imgs'])


def main():
    args = parse_args()
    root_path = args.root_path
    print('Processing training set...')
    num_train_imgs = convert_textocr(
        root_path=root_path,
        dst_image_path='image',
        dst_label_filename='train_label.txt',
        annotation_filename='TextOCR_0.1_train.json',
        nproc=args.n_proc,
        rectify_pose=args.rectify_pose)
    print('Processing validation set...')
    convert_textocr(
        root_path=root_path,
        dst_image_path='image',
        dst_label_filename='val_label.txt',
        annotation_filename='TextOCR_0.1_val.json',
        img_start_idx=num_train_imgs,
        nproc=args.n_proc,
        rectify_pose=args.rectify_pose)
    print('Finish')


if __name__ == '__main__':
    main()
