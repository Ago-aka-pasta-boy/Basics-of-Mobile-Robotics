import numpy as np
import cv2 as cv
import positions as pos

img = cv.imread('img_test.jpg', cv.IMREAD_COLOR)
cv.imshow("image", img)
cv.waitKey(0)

success, goal = pos.get_goal_position(img)
print("\nSuccess is", success, "\nGoal position is:", goal, "\n")


success, robot_position = pos.get_robot_position(img)
print("\nSuccess is", success, "\nRobot position is:", robot_position, "\n")

