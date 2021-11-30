import numpy as np
import cv2 as cv
import vision as vis
import time

# get img from the webcam
#cap_size = (2500, 1500)
#map_shape = [1200,  800]
cap = cv.VideoCapture(0)                                    # put 2 if an external webcam is used
#cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_size[0])               # width
#cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_size[1])              # height
cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)                       # set auto exposure

while True:
    if __debug__:
        print("debugging ON")
    else:
        print("debugging OFF")
    np.imshow("drawing", img)
    time.sleep(1)
    captured, img = cap.read()
    if captured:
        cropped, cropped_img = vis.crop_map(img)
        print(cropped_img.shape)
        if __debug__:
            cv.imshow("Original", img)
        cv.imshow("Cropped", cropped_img)
        if cropped:
            found, obstacles = vis.extract_obstacles(cropped_img)
            if found:
                ex_obstacles = vis.expand_obstacles(obstacles)
                im3 = np.zeros(cropped_img.shape)
                cv.drawContours(im3, ex_obstacles, -1, (0, 255, 0), 3)
                cv.imshow("expanded", im3)
    else:
        print("There was a problem in the capture")
        break
    if (cv.waitKey(1) & 0xFF) == ord('q'):                       # quit when 'Q' key is pressed
        break

cap.release()
cv.destroyAllWindows()