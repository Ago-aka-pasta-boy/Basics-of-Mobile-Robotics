import numpy as np
import cv2 as cv
import math


def detect_circles(img, lower_range, upper_range):
    # convert img from rgb to hsv
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # extract colored circles in white
    mask = cv.inRange(hsv, lower_range, upper_range)
    mask_inv = cv.bitwise_not(mask)
    cv.imshow("image mask", mask)
    cv.waitKey(0)

    # convert img from rgb to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    cv.imshow("image grayscale", gray)
    cv.waitKey(0)

    # apply mask to grayscale image
    img_masked = cv.bitwise_or(gray, gray, mask=mask_inv)
    cv.imshow("image masked", img_masked)
    cv.waitKey(0)

    blurred = cv.blur(img_masked, (3, 3))
    cv.imshow("image blurred", blurred)
    cv.waitKey(0)

    # detect circles
    circles = cv.HoughCircles(blurred, cv.HOUGH_GRADIENT, 1, 20,
                                param1=40, param2=40, minRadius=10, maxRadius=100)

    circles = np.uint16(np.around(circles))
    return circles

# (x,y)


def get_goal_position(img):
    # green color in hsv: 120°, 100%, 50% or 120°, 100%, 100%
    lower_green = np.array([36, 0, 0])
    upper_green = np.array([70, 255, 255])

    circle = detect_circles(img, lower_green, upper_green)

    if circle is None:
        print("no circle found")
        return False, None

    dim = circle.shape
    nb_circles = dim[1]
    print("\ncoordinates of circle is:", circle, "\nnumber of circles detected is:", nb_circles)

    if nb_circles > 1:
        print("number of circles found", nb_circles)
        return False, None

    center = circle[0, 0, 0:2]
    return True, center


# (x,y) + angle between -pi and pi
def get_robot_position(img):
    # blue color in hsv:
    lower_blue = np.array([100, 50, 38])
    upper_blue = np.array([110, 255, 255])

    circles = detect_circles(img, lower_blue, upper_blue)

    list_centers = []
    list_radius = []

    #for i in circles[0, :]:
    #    list_centers.append = (i[0], i[1])
    #    list_radius.append = i[2]

    if not len(circles):
        print("no circle found")
        return 0

    elif len(circles) > 2:
        print("more than two circles found")
        return len(circles)

    #if list_radius[0] > list_radius[1]:
    #    center1 = list_centers[0]
    #    center2 = list_centers[1]
    #else:
    #    center1 = list_centers[1]
    #    center2 = list_centers[0]

    if circles[0][2] > circles[1][2]:
        center1 = circles[0][0:1]
        center2 = circles[1][0:1]
    else:
        center1 = circles[1][0:1]
        center2 = circles[0][0:1]

    angle = math.atan2((center2[1]-center1[1])/(center2[1]-center2[0]))

    position = np.array([center1, angle])

    return position


def get_arch_positions(img):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_red = np.array([169, 100, 100])
    upper_red = np.array([189, 255, 255])

    # extract red arch in white
    mask = cv.inRange(hsv, lower_red, upper_red)
    blurred = cv.blur(mask, (3, 3))



