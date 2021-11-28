import numpy as np
import cv2 as cv

SCALING_FACTOR = 2


def crop_map(img):
    """crop the black borders out of the image to have a clean image to work on"""
    max_crop = 100
    thresh = 200
    crop = 0
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img_blurred = cv.GaussianBlur(img_gray, (5, 5), cv.BORDER_DEFAULT)
    img_filtered = cv.morphologyEx(img_blurred, cv.MORPH_CLOSE, np.ones((5, 5)))    # closing is a dilation followed by
    for i in range(1, max_crop, 5):                                                 # an erosion; it gets rid of small
        crop = i                                                                    # black dots
        cropped = img_filtered[i:-i, i:-i]
        ret, temp = cv.threshold(cropped, thresh, 255, cv.THRESH_BINARY_INV)
        cv.imshow("Thresholded", temp)
        if np.sum(temp[1, :]) < 10 or np.sum(temp[temp.shape[0]-1, :]) < 10 or\
                np.sum(temp[:, 1]) < 10 or np.sum(temp[:, temp.shape[1]-1]) < 10:
            break
    cropped_img = img[crop:-crop, crop:-crop]
    return True, cropped_img


def extract_obstacles(img):
    """Find all the black obstacles and extract their vertices from the image"""
    # cv.imshow("first channel", img[:, :, 0])
    # cv.imshow("second channel", img[:, :, 1])
    # cv.imshow("third channel", img[:, :, 2])
    # RGB input!!
    epsilon = 0.02
    count = []
    thresh = 100
    img_green = img[:, :, 1]
    img_blurred = cv.GaussianBlur(img_green, (5, 5), cv.BORDER_DEFAULT)
    img_filtered = cv.morphologyEx(img_blurred, cv.MORPH_OPEN, np.ones((5, 5)))
    ret, img_ready = cv.threshold(img_filtered, thresh, 255, cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(img_ready, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    im2 = np.zeros(img.shape)
    for count in contours:
        epsilon = 0.01 * cv.arcLength(count, True)
    approximations = cv.approxPolyDP(count, epsilon, closed=True)
    cv.drawContours(im2, [approximations], 0, (0, 255, 0), 3)
    print(approximations[0][0][0])
    # cv.imshow("Contours", im2)
    obstacles = []
    for i in approximations:
        if len(approximations) < 10:
            obstacles.append(approximations[i])
    return True, obstacles


def expand_obstacles(obstacles):
    """expand the size of the obstacles to take the robot size in account for the path"""
    ex_obstacles = np.copy(obstacles)
    for c in obstacles:
        moments = cv.moments(c)
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        for d in obstacles[c]:
            vec_x = obstacles[c][d][0]-cx
            vec_y = obstacles[c][d][1]-cy
            ex_obstacles[c][d][0] = cx + vec_x + SCALING_FACTOR * -1 * (obstacles[c][d][0] > cx)
            ex_obstacles[c][d][1] = cy + vec_y + SCALING_FACTOR * -1 * (obstacles[c][d][1] > cy)
    return ex_obstacles
