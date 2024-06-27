import torch
import numpy as np




def identify_corners(targets):
    '''
    Identify the corners of a rectangle given its four vertices. This function takes a list of targets.
    Each target it is a list with the coorner coordinates in any order
    The output is a list with the targets. The targets are in a dictionary. Sorted into each corners A,B,C and D
    
    Order of corners
    A       B

    C       D
    '''

    targets_sorted = []
    
    for idx, vertices in enumerate (targets):
        if len(vertices) != 4:
            raise ValueError(f"Target {idx} does not contain 4 vertices: {vertices}")
        
        # Sort vertices based on their x-coordinates (left to right)
        sorted_vertices = sorted(vertices, key=lambda vertex: vertex[0])
        # Determine the order by thier y coordinates
        left_vertices = sorted(sorted_vertices[:2], key=lambda vertex: vertex[1])
        right_vertices = sorted(sorted_vertices[2:], key=lambda vertex: vertex[1])
        # Identify top-left and top-right corners
        A, C = left_vertices
    
        # Identify bottom-left and bottom-right corners
        B, D = right_vertices

        targets_sorted.append({'A': A, 'B': B, 'C': C, 'D': D})
    
    return targets_sorted


def point_inside_polygon(x, y, corners):
    '''
    Check if a point (x, y) falls within the polygon defined by four vertices. This function can only accept one target square.
    It only accepts a dictionary with four points

    Order of corners, Q being the point of interest
    A       B
        Q
    C       D
    '''
    
    # Calculate the cross products
    ABxAQ = (corners['D'][0] - corners['A'][0]) * (y - corners['A'][1]) - (corners['B'][1] - corners['A'][1]) * (x - corners['A'][0])
    BDxBQ = (corners['D'][0] - corners['B'][0]) * (y - corners['B'][1]) - (corners['D'][1] - corners['B'][1]) * (x - corners['B'][0])
    CDxDQ = (corners['C'][0] - corners['D'][0]) * (y - corners['D'][1]) - (corners['D'][1] - corners['D'][1]) * (x - corners['D'][0])
    CAxCQ = (corners['A'][0] - corners['C'][0]) * (y - corners['C'][1]) - (corners['A'][1] - corners['C'][1]) * (x - corners['C'][0])

    # Check if the point lies to the left of all sides
    return all(cp >= 0 for cp in [ABxAQ, BDxBQ, CDxDQ, CAxCQ])

def filter_keypoints(feats_no_batch_dim, feats, corners):
    '''
    This function can only accept one target square.
    It only accepts a dictionary with four points
    '''
    
    filtered_features = {
        'keypoints': feats['keypoints'][0][0],
        'keypoint_scores': [feats['keypoint_scores'][0,0]],
        'descriptors': [feats['descriptors'][0,0]],
        'image_size': [feats['image_size'][0]]  # Assuming 'image_size' tensor contains width and height
    }
    
    index_list = []
    
    # Iterate over keypoints without batch dimension
    for idx, keypoint in enumerate(feats_no_batch_dim['keypoints']):
        if point_inside_polygon(keypoint[0], keypoint[1], corners):
            index_list.append(idx)
            # Append corresponding keypoint from feats0_with_batch_dim to filtered_keypoints_with_batch

    filtered_features = {
    'keypoints': torch.unsqueeze(feats['keypoints'][0, index_list], 0),
    'keypoint_scores': torch.unsqueeze(feats['keypoint_scores'][0, index_list], 0),
    'descriptors': torch.unsqueeze(feats['descriptors'][0, index_list], 0),
    'image_size': feats['image_size'][0]  # Assuming 'image_size' tensor contains width and height
    }


    
    return filtered_features
