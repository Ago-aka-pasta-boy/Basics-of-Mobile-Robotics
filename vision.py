import numpy as np
import cv2 as cv


def crop_map(img):
    """crop the black borders out of the image to have a clean image to work on"""
    max_crop = 100
    thresh = 200
    crop = 0
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img_blurred = cv.GaussianBlur(img_gray, (5, 5), cv.BORDER_DEFAULT)
    img_filtered = cv.morphologyEx(img_blurred, cv.MORPH_CLOSE, np.ones((5, 5)))  # closing is a dilation followed by an erosion;
    for i in range(1, max_crop, 5):                                         # it deletes small black points
        crop = i
        cropped = img_filtered[i:-i, i:-i]
        ret, temp = cv.threshold(cropped, thresh, 255, cv.THRESH_BINARY_INV)
        cv.imshow("Thresholded", temp)
        if np.sum(temp[1, :]) < 10 or np.sum(temp[temp.shape[0]-1, :]) < 10 or\
                np.sum(temp[:, 1]) < 10 or np.sum(temp[:, temp.shape[1]-1]) < 10:
            break
    print(crop)
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
