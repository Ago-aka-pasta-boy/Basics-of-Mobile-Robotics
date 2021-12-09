import cv2 as cv
import math

ROBOT_SIZE = 0.11  # in meter
EIFFEL_TOUR_SIZE = 0.14  # in meter
ARCH_SIZE = 0.15  # in meter

def annotate_robot(robot_pos, img, img_thymio, scale_factor):
    """draws the robot on the provided image"""
    # robot dimensions 11 cm x 11cm
    thymio_w = round(ROBOT_SIZE*scale_factor+0.3*ROBOT_SIZE*scale_factor)
    thymio_h = round(ROBOT_SIZE*scale_factor+0.3*ROBOT_SIZE*scale_factor)
    img_thymio = cv.resize(img_thymio, (thymio_w, thymio_h))

    #cv.imshow("Thymio reshaped", img_thymio)
    #cv.waitKey(0)

    # Rotating the thymio
    center = (round(thymio_w/2), round(thymio_h/2))
    rotate_matrix = cv.getRotationMatrix2D(center, math.degrees(robot_pos[1]), scale=1)
    rotated_thymio = cv.warpAffine(img_thymio, rotate_matrix, (thymio_w, thymio_h))

    #cv.imshow("rotated_thymio", rotated_thymio)
    #cv.waitKey(0)

    # x and y bottom left corner of robot in img
    cst = 0.025 * scale_factor  # offset between center of the robot and the center of the wheels
    x = robot_pos[0][0] - center[0] + round(math.cos(robot_pos[1]) * cst)
    y = robot_pos[0][1] - center[1] - round(math.sin(robot_pos[1]) * cst)

    # write thymio on img
    for i in range(rotated_thymio.shape[0]):
        for j in range(rotated_thymio.shape[1]):
            if rotated_thymio[i, j, 3] > 0:
                img[y + i, x + j, :] = rotated_thymio[i, j, :-1]


def annotate_eiffel_tower(goal_pos, img, img_eiffel, scale_factor):
    # goal dimensions diameter = 7cm => eiffel tower height = 14cm
    goal_w = round(EIFFEL_TOUR_SIZE*scale_factor + 0.3 * EIFFEL_TOUR_SIZE*scale_factor)
    goal_h = round(EIFFEL_TOUR_SIZE*scale_factor + 0.3 * EIFFEL_TOUR_SIZE*scale_factor)

    img_eiffel = cv.resize(img_eiffel, (goal_w, goal_h))
    #cv.imshow("Eiffel reshaped", img_eiffel)
    #cv.waitKey(0)

    x = goal_pos[0]-round(goal_w/2)
    y = goal_pos[1]-round(goal_h/2)

    for i in range(img_eiffel.shape[0]):
        for j in range(img_eiffel.shape[1]):
            if img_eiffel[i, j, 3] > 0:
                img[y + i, x + j, :] = img_eiffel[i, j, :-1]


def annotate_arch(arch_pos, img, img_arch, scale_factor):
    # arch dimensions 15 cm
    arch_w = round(ARCH_SIZE * scale_factor + 0.3 * ARCH_SIZE * scale_factor)
    arch_h = round(ARCH_SIZE * scale_factor + 0.3 * ARCH_SIZE * scale_factor)

    img_arch = cv.resize(img_arch, (arch_w, arch_h))
    #cv.imshow("Arch reshaped", img_arch)
    #cv.waitKey(0)

    center_arch = (round(abs(arch_pos[0][0] + arch_pos[1][0])/2), round(abs(arch_pos[0][1] + arch_pos[1][1])/2))
    x = center_arch[0] - round(arch_w/2)
    y = center_arch[1] - round(arch_h/2)

    print("x, y of arch is", x, y)

    # write arch on img
    for i in range(img_arch.shape[0]):
        for j in range(img_arch.shape[1]):
            if img_arch[i, j, 3] > 0:
                img[y + i, x + j, :] = img_arch[i, j, :-1]


