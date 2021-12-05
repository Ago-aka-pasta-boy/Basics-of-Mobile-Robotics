import numpy as np
import cv2 as cv
import math

SCALING_FACTOR = 1.2
ROBOT_LINE = 50


def crop_map(img):
    """crop the black borders out of the image to have a clean image to work on"""
    max_crop = 100
    thresh = 100
    crop = 0
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img_blurred = cv.GaussianBlur(img_gray, (5, 5), cv.BORDER_DEFAULT)
    img_filtered = cv.morphologyEx(img_blurred, cv.MORPH_CLOSE, np.ones((5, 5)))    # closing is a dilation followed by
    for i in range(1, max_crop, 5):                                                 # an erosion; it gets rid of small
        crop = i                                                                    # black dots
        cropped = img_filtered[i:-i, i:-i]
        ret, temp = cv.threshold(cropped, thresh, 255, cv.THRESH_BINARY_INV)
        if __debug__:
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
    thresh = 100
    img_green = img[:, :, 1]
    img_blurred = cv.GaussianBlur(img_green, (5, 5), cv.BORDER_DEFAULT)
    img_filtered = cv.morphologyEx(img_blurred, cv.MORPH_OPEN, np.ones((5, 5)))
    ret, img_ready = cv.threshold(img_filtered, thresh, 255, cv.THRESH_BINARY)
    cv.imshow("Thresholded", img_ready)
    contours, hierarchy = cv.findContours(img_ready, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    im2 = np.zeros(img.shape)
    approximations = []
    for count in range(len(contours)):
        epsilon = 0.03 * cv.arcLength(contours[count], True)
        if (cv.arcLength(contours[count], True) < 1000) & (cv.arcLength(contours[count], True) > 200):
            approximations.append(cv.approxPolyDP(contours[count], epsilon, closed=True))
    cv.drawContours(im2, approximations, -1, (0, 255, 0), 3)
    cv.imshow("Contours", im2)
    obstacles = []
    for i in range(len(approximations)):
        if __debug__:
            print("approx", i, len(approximations[i]))
        if len(approximations[i]) < 5:
            obstacles.append(approximations[i])
    im3 = np.zeros(img.shape)
    cv.drawContours(im3, obstacles, -1, (0, 255, 0), 3)
    cv.imshow("obstacles", im3)
    if not obstacles:
        return False, []
    return True, obstacles


def expand_obstacles(obstacles):
    """expand the size of the obstacles to take the robot size in account for the path"""
    ex_obstacles = np.copy(obstacles)
    for c in range(len(obstacles)):
        moments = cv.moments(obstacles[c])
        if moments["m00"] == 0:
            moments["m00"] = 0.01
        if moments["m00"] == 0:
            moments["m00"] = 0.01
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        for d in range(len(obstacles[c])):
            vec_x = obstacles[c][d][0][0]-cx
            vec_y = obstacles[c][d][0][1]-cy
            ex_obstacles[c][d][0][0] = cx + vec_x * SCALING_FACTOR
            ex_obstacles[c][d][0][1] = cy + vec_y * SCALING_FACTOR
    return ex_obstacles


def annotate_path(path, img):
    """draws a line from start to end on the provided image"""
    for vert in path-1:
        cv.line(img, path[vert], path[vert+1], (255, 255, 255), 3)


def annotate_robot(robot_pos, img):
    """draws the robot on the provided image"""
    cv.circle(img, robot_pos[0], 10, (0, 0, 255), -1)
    point2 = (int(robot_pos[0][0]+math.cos(robot_pos[1])*ROBOT_LINE), int(robot_pos[0][1]-math.sin(robot_pos[1])*ROBOT_LINE))
    cv.line(img, robot_pos[0], point2, (0, 0, 255), 5)
    cv.putText(img, "Thymio", robot_pos[0], cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)


def annotate_goal(goal_pos, img):
    """draws the goal on the provided image"""
    cv.circle(img, (goal_pos[0], goal_pos[1]), goal_pos[2], (0, 255, 0), -1)
    cv.putText(img, "Goal", (goal_pos[0], goal_pos[1]), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)


def annotate_arch(arch_pos, img):
    """draws the way through the arch on the provided image"""
    point1 = arch_pos[len(arch_pos)-2]
    point2 = arch_pos[len(arch_pos)-1]
    cv.line(img, point1, point2, (0, 0, 255), -1)
    cv.putText(img, "arch", ((arch_pos[0][0]+arch_pos[0][1])/2, arch_pos[1]), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)


def convert_vertice(obstacles):
    """convert the base listing of opencv for contours into an array of arrays of tuples used in global navigation"""
    converted_vertice = []
    conv = []
    for c in range(len(obstacles)):
        for d in range(len(obstacles[c])):
            conv.append(tuple(obstacles[c][d][0]))
        converted_vertice.append(conv)
    return converted_vertice
