import os
import matplotlib.pyplot as plt
import cv2 as cv
import numpy as np

from lightglue import LightGlue, SuperPoint, DISK
from lightglue.utils import load_image, rbd
from lightglue import viz2d
import torch

torch.set_grad_enabled(False)



# This function matches a query image to the perspective of the base image. It does this using superpoint feature extractor and lightglue feature matcher
def match_images_and_transform(base_image_path, query_image_path, feats_base_image):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # Check if Nvidia CUDA is supported by the gpu otherwise set device to cpu

    
    extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)  # Load Superpoint as the extractor
    matcher = LightGlue(features="superpoint").eval().to(device) # Load Lightglue as the matcher

    # Load the base and the query image from the paths
    base_image = load_image(base_image_path)
    query_image = load_image(query_image_path)

    # Extract features in both images and store them on the device
    feats_query_image = extractor.extract(query_image.to(device))

    # If the feats of the base image are not calculated before calling the function it should extract the features
    if feats_base_image == None:
        feats_base_image = extractor.extract(base_image.to(device))

    # Match the features found. Also remove the batch dimention for further processing
    matches_batch_dim = matcher({"image0": feats_base_image, "image1": feats_query_image})
    feats_base_image_no_batch_dim, feats_query_image_no_batch_dim, matches_no_batch_dim = [
        rbd(x) for x in [feats_base_image, feats_query_image, matches_batch_dim]
    ]  # remove batch dimension

    # Take out the keypoints from the features
    kpts_base, kpts_query, matches = feats_base_image_no_batch_dim["keypoints"], feats_query_image_no_batch_dim["keypoints"], matches_no_batch_dim["matches"]
    # We are only interested in the matching keypoints
    m_kpts_base, m_kpts_query = kpts_base[matches[..., 0]], kpts_query[matches[..., 1]]


    # Load the images with openCV for further processing
    CV_image0 = cv.imread(base_image_path)
    CV_image1 = cv.imread(query_image_path)
    
    # Homography estimation
    
    homography_result = cv.findHomography(m_kpts_query.cpu().numpy(), m_kpts_base.cpu().numpy(), cv.RANSAC, 5.0)
    
    transformed_image = cv.warpPerspective(CV_image1, homography_result[0], (CV_image0.shape[1], CV_image0.shape[0]))

    return(transformed_image)




def extract_image(image, corners): # Takes the image and the corners in a numpy array as input and returns the image with corrected perspective.
     # Define the perfect square corners
    perfect_square = np.array([[0, 0], [0, 500], [500, 500], [500, 0]], dtype=np.float32)

    # Convert the corner points to numpy array
    corners = np.array(corners, dtype=np.float32)

    # Calculate the perspective transformation matrix
    transform_matix = cv.getPerspectiveTransform(corners, perfect_square)

    # Apply the perspective transformation to the image
    corrected_image = cv.warpPerspective(image, transform_matix, (500, 500))  # Adjust the size as needed

    corrected_image = np.transpose(corrected_image, (1,0,2)) # Flips the image into portrait mode

    return corrected_image




def save_images(result, image_path, base_image_path, targets, save_path, bag, total_images_saved, feats_base_image, print_info):
    amount_images = 0 # Variable for counting the amount of images that have been saved from this bag.
    for idx, element in enumerate(result): # Go though the result one by one
        image = cv.imread(image_path + element['img']) #Load the image file with openCV
    
        # Note: The corners are as follows; C1: bottom right, C2: top right, C3: top left, C4: bottom left
        transformed_image = match_images_and_transform(base_image_path, image_path + element['img'], feats_base_image)

        for idx, corners in enumerate(targets):
            extracted_image = extract_image(transformed_image, [corners['A'], corners['B'], corners['D'], corners['C']]) # Extract the image patches from the image
            
            save_directory = (save_path + bag + '.patches' '/target_' + str(idx) + '/') # Use the specified save_path for where to save the file. Add the bag of origin and also specify that it contains patches and not the bag iself
            if not os.path.exists(save_directory): # If the directory does not exist, creake it
                os.makedirs(save_directory)
                print('Made directory ' + save_directory) # Inform the user that the dirctory has been made
    
            # If the user requested that the image data is printed, print it
            if print_info == True: print('--Extracted patch from image'+ element['img'] + '" in directory (' + image_path + ') and saved it as ' + 'patch_' + element['img'] + ' in directory (' + save_directory + ')') 
            
            amount_images += 1 # Itterate the counter that counts the amount of images saved.
            
            # Save the image to the save directory, naming it its original name with patch addad as a prefix
            cv.imwrite(save_directory + 'targetpatch_' + str(idx) + '_' + element['img'], extracted_image) 
        
    # Printing information about the operation done
    print('Saved ' + str(amount_images) + ' images')
    print('Done extracting and saving image patches from bag\n')




    total_images_saved += amount_images
    return(total_images_saved)