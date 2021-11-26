import numpy as np
import cv2 as cv
import vision as vis
import time

# get img from the webcam
cap_size = (1200, 1000)
map_shape = [1200,  800]
cap = cv.VideoCapture(1)                                    # put 1 if an external webcam is used
cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_size[0])               # width
cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_size[1])              # height
cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)                       # set auto exposure

while True:
    captured, img = cap.read()
    if captured:
        cropped, cropped_img = vis.crop_map(img)
        cv.imshow("Original", img)
        cv.imshow("Cropped", cropped_img)
        if cropped:
            found, obstacles = vis.extract_obstacles(img)
            if found:
                ex_obstacles = vis.expand_obstacles(obstacles)
    else:
        print("There was a problem in the capture")
        break
    if (cv.waitKey(1) & 0xFF) == ord('q'):                       # quit when 'Q' key is pressed
        break

cap.release()
cv.destroyAllWindows()