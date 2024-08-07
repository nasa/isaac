import numpy as np


def duplicate(p1, p2, limit=0.05):
    """
    Given two locations of labels, return true if the two labels are considered "duplicates" based
    specified limit.

    @param p1       array representing 3D position of label
    @param p2       array representing 3D position of label
    @param limit    max distance to be considered same location
    """

    pos1 = np.array(p1[:3])
    pos2 = np.array(p2[:3])

    return np.linalg.norm(pos2 - pos1) < limit


def get_euclidean_distance(p1, p2):
    """
    @param p1       array representing 3D position of label
    @param p2       array representing 3D position of label
    @returns        euclidean 2D distance between p1 and p2
    """

    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def get_closest_rect(rect, rectangles, positions, distance):
    """
    @param rect         2D array representing rectangle (upper left and lower right coordinates)
    @param rectangles   array of rectangles
    @param positions    3D positions of the rectangles
    @param distance     upper limit of distance to get close rectangles
    @returns            rects array of rectangles within 'distance' of 'rect'
                        pos array of the positions of the close rectangles
    """

    rects = []
    pos = []
    for i in range(len(rectangles)):
        d = get_rect_distance(rect[0], rect[1], rectangles[i][0], rectangles[i][1])
        if d < distance:
            rects.append(rectangles[i])
            if positions is not None:
                pos.append(positions[i])
            else:
                pos = None
    return rects, pos


def get_iou(rect1, rect2):
    """
    Calculates the intersection over union given two rectangles in the form of [(upper_left, lower_right)] coordinates.

    @param rect1    2D array representing rectangle (upper left and lower right coordinates)
    @param rect2    2D array representing rectangle (upper left and lower right coordinates)
    @returns        iou of the two rectangles
    """

    x1_ul, y1_ul = rect1[0]
    x1_lr, y1_lr = rect1[1]
    x2_ul, y2_ul = rect2[0]
    x2_lr, y2_lr = rect2[1]

    # Calculate the coordinates of the intersection rectangle
    intersection_x_ul = max(x1_ul, x2_ul)
    intersection_y_ul = max(y1_ul, y2_ul)
    intersection_x_lr = min(x1_lr, x2_lr)
    intersection_y_lr = min(y1_lr, y2_lr)

    # Calculate the width and height of the intersection rectangle
    intersection_w = max(0, intersection_x_lr - intersection_x_ul)
    intersection_h = max(0, intersection_y_lr - intersection_y_ul)

    # Calculate the area of intersection
    intersection_area = intersection_w * intersection_h

    # Calculate the area of each rectangle
    rect1_area = (x1_lr - x1_ul) * (y1_lr - y1_ul)
    rect2_area = (x2_lr - x2_ul) * (y2_lr - y2_ul)

    # Calculate the union area
    union_area = rect1_area + rect2_area - intersection_area

    # Calculate the IoU
    iou = intersection_area / union_area
    # iou = intersection_area / min(rect1_area, rect2_area) # iou between the smaller area
    return iou


def overlap(rect1, rect2):
    """
    @param rect1    2D array representing rectangle (upper left and lower right coordinates)
    @param rect2    2D array representing rectangle (upper left and lower right coordinates)
    @returns        true if rect1 and rect2 overlap
    """

    return get_iou(rect1, rect2) > 0.5


def get_bounding_box(rect1, rect2):
    """
    @param rect1    2D array representing rectangle (upper left and lower right coordinates)
    @param rect2    2D array representing rectangle (upper left and lower right coordinates)
    @returns        rectangle encasing rect1 and rect2
    """

    upper_left = np.array(
        (min(rect1[0][0], rect2[0][0]), min(rect1[0][1], rect2[0][1]))
    )
    lower_right = np.array(
        (max(rect1[1][0], rect2[1][0]), max(rect1[1][1], rect2[1][1]))
    )

    return np.array((upper_left, lower_right))


def get_midpoint(pos1, pos2):
    """
    @param pos1     3D position
    @param pos2     3D position
    @returns the midpoint between pos1 and pos2
    """

    return [(pos1[i] + pos2[i]) / 2 for i in range(len(pos1))]


def get_rect_distance(upper_a, lower_a, upper_b, lower_b):
    """
    @param upper_a  upper left coordinate of rect 1
    @param lower_a  lower right coordinate of rect 1
    @param upper_b  upper left coordinate of rect 2
    @param lower_b  lower right coordinate of rect 2
    @returns distance between rect 1 and rect 2
    """

    x1, y1 = upper_a
    x1b, y1b = lower_a
    x2, y2 = upper_b
    x2b, y2b = lower_b

    left = x2b < x1
    right = x1b < x2
    bottom = y2b < y1
    top = y1b < y2
    if top and left:
        return get_euclidean_distance((x1, y1b), (x2b, y2))
    elif left and bottom:
        return get_euclidean_distance((x1, y1), (x2b, y2b))
    elif bottom and right:
        return get_euclidean_distance((x1b, y1), (x2, y2b))
    elif right and top:
        return get_euclidean_distance((x1b, y1b), (x2, y2))
    elif left:
        return x1 - x2b
    elif right:
        return x2 - x1b
    elif bottom:
        return y1 - y2b
    elif top:
        return y2 - y1b
    else:  # rectangles intersect
        return 0.0


def crop_image(img, startx, starty, endx, endy):
    """
    @param img      array of image
    @param startx   start x pixel
    @param starty   start y pixel
    @param endx     end x pixel
    @param endy     end y pixel
    @returns a cropped image of img[startx:endx, starty:endy]
    """
    h, w, _ = img.shape

    startx = max(0, startx)
    starty = max(0, starty)

    endx = min(endx, w)
    endy = min(endy, h)

    return img[starty:endy, startx:endx]
