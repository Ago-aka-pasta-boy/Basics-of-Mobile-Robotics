import numpy as np
import cv2 as cv
import math


def detect_circles(img, lower_range, upper_range):
    # convert img from rgb to hsv
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # threshold colored circles
    mask = cv.inRange(hsv, lower_range, upper_range)
    mask_inv = cv.bitwise_not(mask)

    # convert img from rgb to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # apply threshold mask to grayscale image
    img_masked = cv.bitwise_and(gray, gray, mask=mask)

    # blur image
    blurred = cv.blur(img_masked, (5, 5))
    # cv.imshow("image blurred", blurred)
    # cv.waitKey(0)
    # cv.destroyAllWindows()

    # detect circles
    circles = cv.HoughCircles(blurred, cv.HOUGH_GRADIENT, 1, 20,
                                param1=50, param2=30, minRadius=3, maxRadius=100)
    if circles is None:
        return None

    # draw circles
    output = cv.cvtColor(gray, cv.COLOR_GRAY2BGR)
    circles = np.uint16(np.around(circles))
    for i in circles[0, :]:
        # draw the outer circle
        cv.circle(output, (i[0], i[1]), i[2], (0, 255, 0), 2)
        # draw the center of the circle
        cv.circle(output, (i[0], i[1]), 2, (0, 0, 255), 3)

    # cv.imshow("detected circles", output)
    # cv.waitKey(0)
    # cv.destroyAllWindows()

    # convert the (x, y) coordinates and radius of the circles to integers
    circles = np.round(circles[0, :]).astype("int")

    return circles

""" return position of the goal (x,y)"""
def get_goal_position(img):
    # color value for green in hsv
    lower_green = np.array([35, 50, 0])
    upper_green = np.array([75, 255, 255])

    # detect circle
    circle = detect_circles(img, lower_green, upper_green)

    if circle is None:
        print("WARNING: no goal position found")
        return False, None

    dim = circle.shape
    nb_circles = dim[0]
    #print("\ncoordinates of goal position is:", circle, "\nnumber of goal detected is:", nb_circles)

    if nb_circles > 1:
        print("WARNING: more than one goal position found", nb_circles)
        return False, None

    center = circle[0, 0:2]
    return True, center


""" return robot position (x,y) + angle between -pi and pi"""
def get_robot_position(img):
    # mask for blue color in hsv:
    lower_blue = np.array([85, 50, 0])
    upper_blue = np.array([130, 255, 255])

    circles = detect_circles(img, lower_blue, upper_blue)

    if circles is None:
        print("WARNING: robot not found")
        return False, None

    dim = circles.shape
    nb_circles = dim[0]
    #print("\ncoordinates of circle is:", circles, "\nnumber of circles detected is:", nb_circles)

    if (nb_circles > 2) or (nb_circles == 1):
        print("WARNING: more than 2 circles found for robot position", nb_circles)
        return False, None

    # direction of the robot
    if nb_circles == 2:
        if circles[0, 2] > circles[1, 2]:
            point = circles[0, 0:2]
            dy = circles[0,1]-circles[1,1]
            dx = circles[1,0]-circles[0,0]

        else:
            point = circles[1, 0:2]
            dy = circles[1, 1] - circles[0, 1]
            dx = circles[0, 0] - circles[1, 0]

        angle = math.atan2(dy, dx)
        position = [point, angle]
        return True, position


def get_arch_positions(img):
    # extract red arch in white with threshold
    mask = extract_red(img)

    # blur image
    blurred = cv.blur(mask, (5, 5))

    #cv.imshow("image blurred", blurred)
    #cv.waitKey(0)
    #cv.destroyAllWindows()

    # detect rectangle
    c, hierarchy = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

    approx = []
    for i in range(len(c)):
        epsilon = 0.03 * cv.arcLength(c[i], True)
        if (cv.arcLength(c[i], True) < 1000) & (cv.arcLength(c[i], True) > 200):
            approx.append(cv.approxPolyDP(c[i], epsilon, closed=True))

    if not approx:
        print("WARNING: no red shape detected")
        return False, None

    elif len(approx) > 1:
        print("WARNING: more than one red shape detected")
        return False, None

    # check if shape is a rectangle
    if len(approx[0]) == 4:
        M = cv.moments(approx[0])
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        mu11 = M["m11"]/M["m00"]-cx*cy
        mu02 = M["m02"]/M["m00"]-math.pow(cy,2)
        mu20 = M["m20"]/M["m00"]-math.pow(cx,2)

        angle = -0.5*math.atan2(2*mu11,mu20-mu02)
        #print("\nangle is", angle)

    else:
        print("WARNING: no rectangle found")
        return False, None

    # draw center
    center = np.array([cx, cy], dtype=int)
    cv.circle(img, center, 2, (0, 0, 0), 3)

    # find point1 and point2
    width = min(math.dist(approx[0][0][0], approx[0][1][0]), math.dist(approx[0][1][0], approx[0][2][0]))
    point1 = np.array([cx + math.sin(angle)*width, cy + math.cos(angle)*width], dtype=int)
    point2 = np.array([cx - math.sin(angle)*width, cy - math.cos(angle)*width], dtype=int)

    # draw points on img
    cv.circle(img, point1, 2, (0, 0, 0), 3)
    cv.circle(img, point2, 2, (0, 0, 0), 3)
    cv.circle(img, center, 2, (0, 0, 0), 3)

    # show center and two positions on image
    #cv.imshow("point1 and point2 of rectangle", img)
    #cv.waitKey(0)
    #cv.destroyAllWindows()

    positions = point1, point2

    return True, positions


def extract_red(img):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # mask detection in hsv for red color
    lower_red1 = np.array([0, 70, 0])
    upper_red1 = np.array([15, 255, 255])

    lower_red2 = np.array([170, 70, 0])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv.inRange(hsv, lower_red2, upper_red2)

    mask = mask1 | mask2

    return mask



