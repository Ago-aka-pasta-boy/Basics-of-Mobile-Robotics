import numpy as np
import cv2 as cv


def crop_map(img):
    """crop the black borders out of the image to have a clean image to work on"""
    max_crop = 10
    thresh = 50
    crop = 0
    for i in range(max_crop):
        crop = i
        cropped = img[i:-i, i:-i]
        temp = cv.threshold(cropped, thresh, 1, cv.THRESH_BINARY_INV)
        if temp[1, :].sum() > 10 or temp[temp.size(1), :].sum() > 10 or\
                temp[:, 1].sum() > 10 or temp[:, temp.size(2)].sum() > 10:
            break
    cropped_img = img[crop:-crop, crop:-crop]
    return True, cropped_img


def extract_obstacles(img):
    """Find all the black obstacles and extract their vertices from the image"""
    obstacles = []
    return True, obstacles


def expand_obstacles(obstacles):
    """expand the size of the obstacles to take the robot size in account for the path"""
    ex_obstacles = np.copy(obstacles)
    return ex_obstacles
