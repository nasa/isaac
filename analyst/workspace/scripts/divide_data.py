import numpy as np
import os
import shutil
import random
import cv2 as cv



def make_directories(train_test_dir, targets):
    train_test_dir_original = train_test_dir
    
    directories_made = []
    
    for i in range(targets):
        train_dir = train_test_dir_original + 'target_' + str(i) + '/train'
        test_dir = train_test_dir_original + 'target_' + str(i) + '/test'
        
        # Remove if old training/testing data exists
        if os.path.exists(train_dir):
            shutil.rmtree(train_dir) 
        if os.path.exists(test_dir):
            shutil.rmtree(test_dir) 
    
        # Create directories for training and testing data if they don't exist
        os.makedirs(train_dir + '/on', exist_ok=True)
        os.makedirs(test_dir + '/on', exist_ok=True)
        os.makedirs(train_dir + '/off', exist_ok=True)
        os.makedirs(test_dir + '/off', exist_ok=True)

        directories_made.append([train_dir, test_dir])

    return directories_made

def divide_data(bag_case, save_path, directories_made, amount_of_training_data):
    totnr = 0
    for target, directories in enumerate(directories_made):
        train_dir = directories[0]
        test_dir = directories[1]

        
        for bagfile, case in bag_case.items(): # Go through the bagfile case list
            bag_patch_path = os.path.join(save_path + '/' + bagfile, 'target_' + str(target)) # path to current bag file
            
            #if not bag_patch_path.endswith('.patches'): # Must end with .patches
                #continue
        
            images = os.listdir(bag_patch_path) # List all the image patches that are saved
        
            if case == 'on': # If the case for the current bag is that the switch was on, sets the training and testing paths accordingly
                train_path = os.path.join(train_dir, 'on')
                test_path = os.path.join(test_dir, 'on')
            elif case == 'off': # Same as for the on case but in the case the switch was set to off for the bag
                train_path = os.path.join(train_dir, 'off')
                test_path = os.path.join(test_dir, 'off')
            else: # Incase the bag has not been specified it prints that for the user and continues with the next one
                print('Case for ' + bagfile + ' is not specified as on or off')
                continue
        
            # Goes through all of the pictues and moves them to the approriate path, the data is split up into testing and training data in accordance with the specified split
            for idx, image_name in enumerate(images):
                image_data = cv.imread(os.path.join(bag_patch_path, image_name))
                if not image_name.endswith('.jpg') is True: # Added to avoid error related to jupyternotebook checkpoint file
                    continue
                if random.uniform(0,1) > amount_of_training_data: # Check if the image should be in the training or test data set
                    cv.imwrite(os.path.join(test_path, image_name), image_data)
                else:
                    cv.imwrite(os.path.join(train_path, image_name), image_data)
                    
        # This secion checks how much data is in the training and testing data set and informs the user
        
        trainnr = len(os.listdir(os.path.join(train_dir, 'on')))
        trainnr += len(os.listdir(os.path.join(train_dir, 'off')))
        testnr = len(os.listdir(os.path.join(test_dir, 'on')))
        testnr += len(os.listdir(os.path.join(test_dir, 'off')))
        totnr = totnr + trainnr + testnr
        
    print('Data copying and organization completed.\n')
    print('Amount of total data is: ' + str(totnr))
    print('Amount of training data is: ' + str(trainnr)) #str(len(os.listdir(train_dir))))
    print('Amount of test data is: ' + str(testnr)) #str(len(os.listdir(test_dir))))