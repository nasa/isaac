from lightglue import LightGlue, SuperPoint, DISK
from lightglue.utils import load_image, rbd
from lightglue import viz2d
import torch

torch.set_grad_enabled(False)

import matplotlib.pyplot as plt
import cv2 as cv
import numpy as np



def match_images_and_crop(base_image_path, query_image_path):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 'mps', 'cpu'
    
    extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)  # load the extractor
    matcher = LightGlue(features="superpoint").eval().to(device)


    base_image = load_image(base_image_path)
    query_image = load_image(query_image_path)
    
    feats_base_image = extractor.extract(base_image.to(device))
    feats_query_image = extractor.extract(query_image.to(device))
    matches_batch_dim = matcher({"image0": feats_base_image, "image1": feats_query_image})
    feats_base_image_no_batch_dim, feats_query_image_no_batch_dim, matches_no_batch_dim = [
        rbd(x) for x in [feats_base_image, feats_query_image, matches_batch_dim]
    ]  # remove batch dimension
    
    kpts_base, kpts_query, matches = feats_base_image_no_batch_dim["keypoints"], feats_query_image_no_batch_dim["keypoints"], matches_no_batch_dim["matches"]
    m_kpts_base, m_kpts_query = kpts_base[matches[..., 0]], kpts_query[matches[..., 1]]

    CV_image0 = cv.imread(base_image_path)
    CV_image1 = cv.imread(query_image_path)
    
    # Homography estimation
    
    homography_result = cv.findHomography(m_kpts_query.numpy(), m_kpts_base.numpy(), cv.RANSAC, 5.0)
    
    transformed_image = cv.warpPerspective(CV_image1, homography_result[0], (CV_image0.shape[1], CV_image0.shape[0]))

    return(transformed_image)

