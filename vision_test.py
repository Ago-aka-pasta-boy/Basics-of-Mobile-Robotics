import numpy as np
import cv2 as cv
import vision as vis
import positions as pos
import time

# get img from the webcam
# cap_size = (2500, 1500)
# map_shape = [1200,  800]
cap = cv.VideoCapture(0)                                    # put 2 if an external webcam is used
# cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_size[0])               # width
# cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_size[1])              # height
cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)                       # set auto exposure

while True:
    time.sleep(0.5)
    captured, img = cap.read()
    if captured:
        cropped, cropped_img = vis.crop_map(img)
        # print(cropped_img.shape)        # uncomment to debug
        # cv.imshow("Original", img)        # uncomment to debug
        cv.imshow("Cropped", cropped_img)
        if cropped:
            im3 = np.zeros(cropped_img.shape)
            found, obstacles = vis.extract_obstacles(cropped_img)
            if found:
                ex_obstacles = vis.expand_obstacles(obstacles)
                cv.drawContours(im3, ex_obstacles, -1, (0, 255, 0), 3)
                cv.imshow("expanded", im3)
            found_rob, robot_pos = pos.get_robot_position(cropped_img)
            if found_rob:
                vis.annotate_robot(robot_pos, im3)
            found_goal, goal_pos = pos.get_goal_position(cropped_img)
            if found_goal:
                vis.annotate_goal(goal_pos, im3)
            found_arch, arch_pos = pos.get_arch_positions(cropped_img)
            if found_arch:
                vis.annotate_arch(arch_pos, im3)
            cv.imshow("environment", im3)
    else:
        print("There was a problem in the capture")
        break
    if (cv.waitKey(1) & 0xFF) == ord('q'):                       # quit when 'Q' key is pressed
        break

cap.release()
cv.destroyAllWindows()
