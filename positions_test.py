import numpy as np
import cv2 as cv
import positions as pos
import matplotlib.pyplot as plt

img = cv.imread('img_test.png', cv.IMREAD_UNCHANGED).astype('float32')
cv.imshow("image", img)

success, goal = pos.get_goal_position(img)

print("\n Succes", success, "Goal position:", goal, "\n")

