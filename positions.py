import numpy as np
import cv2 as cv
import math


def detect_circles(img, lower_range, upper_range):
    # convert img from rgb to hsv
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # threshold colored circles
    mask = cv.inRange(hsv, lower_range, upper_range)
    mask_inv = cv.bitwise_not(mask)
    cv.imshow("image mask", mask)
    cv.waitKey(0)
    cv.destroyAllWindows()

    # convert img from rgb to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    cv.imshow("image grayscale", gray)
    cv.waitKey(0)
    cv.destroyAllWindows()

    # apply threshold mask to grayscale image
    img_masked = cv.bitwise_and(gray, gray, mask=mask)
    cv.imshow("image masked", img_masked)
    cv.waitKey(0)
    cv.destroyAllWindows()

    blurred = cv.blur(img_masked, (5, 5))
    cv.imshow("image blurred", blurred)
    cv.waitKey(0)
    cv.destroyAllWindows()

    # detect circles
    circles = cv.HoughCircles(blurred, cv.HOUGH_GRADIENT, 1, 20,
                                param1=50, param2=30, minRadius=5, maxRadius=100)
    if circles is None:
        print("no circle found")
        return None

    # draw circles
    """"    
    output = img.copy
    
    for (x, y, r) in circles:
        radius_map[r].append((x, y, r))
        
    for x, y, r in circles[0,:]:
        cv.circle(output, (x, y), r, (0, 255, 0), 4)
        cv.imshow("circles in green", output)
    """
    """
    for (x, y, r) in circles:
        radius_map[r].append((x, y, r))
    
    for key in radius_map:
        if len(radius_map[key]) > 0:
            output = img.copy()
            for x, y, r in radius_map[key]:
                cv.circle(output, (x, y), r, (0, 255, 0), 4)
                cv.imshow(f"Radius {key}", output)
    """

    output = cv.cvtColor(gray, cv.COLOR_GRAY2BGR)
    circles = np.uint16(np.around(circles))
    for i in circles[0, :]:
        # draw the outer circle
        cv.circle(output, (i[0], i[1]), i[2], (0, 255, 0), 2)
        # draw the center of the circle
        cv.circle(output, (i[0], i[1]), 2, (0, 0, 255), 3)

    cv.imshow("detected circles", output)
    cv.waitKey(0)
    cv.destroyAllWindows()

    # convert the (x, y) coordinates and radius of the circles to integers
    circles = np.round(circles[0, :]).astype("int")

    return circles

# return position of the goal (x,y)
def get_goal_position(img):
    # green color in hsv: 120°, 100%, 50% or 120°, 100%, 100%
    lower_green = np.array([35, 50, 0])
    upper_green = np.array([75, 255, 255])

    circle = detect_circles(img, lower_green, upper_green)

    if circle is None:
        print("no circle found")
        return False, None

    dim = circle.shape
    nb_circles = dim[0]
    print("\ncoordinates of circle is:", circle, "\nnumber of circles detected is:", nb_circles)

    if nb_circles > 1:
        print("number of circles found", nb_circles)
        return False, None

    center = circle[0, 0:2]
    return True, center


# return robot position (x,y) + angle between -pi and pi
def get_robot_position(img):
    # mask for blue color in hsv:
    lower_blue = np.array([85, 50, 0])
    upper_blue = np.array([130, 255, 255])

    circles = detect_circles(img, lower_blue, upper_blue)

    if circles is None:
        print("no circle found")
        return False, None

    dim = circles.shape
    nb_circles = dim[0]
    print("\ncoordinates of circle is:", circles, "\nnumber of circles detected is:", nb_circles)

    if nb_circles > 2:
        print("number of circles found", nb_circles)
        return False, None

    # direction of the robot
    if nb_circles == 2:
        if circles[0, 2] > circles[1, 2]:
            point = circles[0, 0:2]
            dy = circles[0,1]-circles[1,1]
            dx = circles[1,0]-circles[0,0]
            print(dy, dx)
        else:
            angle = math.atan2(circles[0,1]-circles[1,1], circles[1,0]-circles[0,0])
            dy = circles[0,1]-circles[1,1]
            dx = circles[0,0]-circles[1,0]
            print(dy, dx)

    angle = math.atan2(dy, dx)
    position = np.array([point, angle])

    return True, position


def get_arch_positions(img):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_red = np.array([169, 100, 100])
    upper_red = np.array([189, 255, 255])

    # extract red arch in white
    mask = cv.inRange(hsv, lower_red, upper_red)
    blurred = cv.blur(mask, (3, 3))



