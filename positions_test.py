import numpy as np
import cv2 as cv
import positions as pos
import vision
import drawing as draw

img = cv.imread('img_test_final.jpg', cv.IMREAD_COLOR)
cv.imshow("image", img)
cv.waitKey(0)

img_thymio = cv.imread('thymio.png', cv.IMREAD_UNCHANGED)
img_eiffel = cv.imread('tour_eiffel.png', cv.IMREAD_UNCHANGED)
img_arch = cv.imread('arch.png', cv.IMREAD_UNCHANGED)

success, goal, radius = pos.get_goal_position(img)
print("\nSuccess is", success, "\nGoal position is:", goal, "\nradius", radius)

factor = pos.convert_meter2pxl(radius)
print("\nConversion factor meter to pixel", factor)


success, robot_position = pos.get_robot_position(img)
print("\nSuccess is", success, "\nRobot position is:", robot_position, "\n")

success, positions = pos.get_arch_positions(img)
print("\nSuccess is", success, "\nTwo positions", positions)


print("dimension of image", img.shape, "\nx, y of image", img.shape[0], img.shape[1])
img2 = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
print("dimension of image 2", img2.shape, "\nx, y of image", img2.shape[0], img2.shape[1])

draw.annotate_robot(robot_position, img2, img_thymio, factor)
draw.annotate_eiffel_tower(goal, img2, img_eiffel, factor)
draw.annotate_arch(positions, img2, img_arch, factor)


cv.imshow("Image with drawings", img)
cv.waitKey(0)

cv.imshow("image zero", img2)
cv.waitKey(0)
