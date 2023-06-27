import numpy as np


def get_euclidean_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def get_rect_distance(upper_a, lower_a, upper_b, lower_b):
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
