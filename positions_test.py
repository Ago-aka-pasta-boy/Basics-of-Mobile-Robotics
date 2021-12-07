import numpy as np
import cv2 as cv
import positions as pos

img = cv.imread('img_test_final.jpg', cv.IMREAD_COLOR)
cv.imshow("image", img)
cv.waitKey(0)

success, goal, radius = pos.get_goal_position(img)
print("\nSuccess is", success, "\nGoal position is:", goal, "\nradius", radius)

factor = pos.convert_meter2pxl(radius)
print("\nConversion factor meter to pixel", factor)


success, robot_position = pos.get_robot_position(img)
print("\nSuccess is", success, "\nRobot position is:", robot_position, "\n")

success, positions = pos.get_arch_positions(img)
print("\nSuccess is", success, "\nTwo positions", positions)