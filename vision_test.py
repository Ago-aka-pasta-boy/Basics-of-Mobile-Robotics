import numpy as np
import cv2 as cv
import vision as vis
import positions as pos

# get img from the webcam
cap = cv.VideoCapture(0)                                    # put 2 if an external webcam is used
cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)                       # set auto exposure

while True:
    captured, img = cap.read()
    if captured:
        cv.imshow("Original", img)
        im3 = np.zeros(img.shape)
        found, obstacles = vis.extract_obstacles(img)
        if found:
            ex_obstacles = vis.expand_obstacles(obstacles)
            cv.drawContours(im3, ex_obstacles, -1, (0, 255, 0), 3)
            cv.imshow("expanded", im3)
        found_rob, robot_pos = pos.get_robot_position(img)
        if found_rob:
            vis.annotate_robot(robot_pos, im3)
        found_goal, goal_pos, ignore = pos.get_goal_position(img)
        if found_goal:
            vis.annotate_goal(goal_pos, im3)
        found_arch, arch_pos = pos.get_arch_positions(img)
        if found_arch:
            vis.annotate_arch(arch_pos, im3)
        cv.imshow("environment", im3)
    else:
        print("There was a problem in the capture")
        break
    if (cv.waitKey(1) & 0xFF) == ord('c'):  # press c to quit
        break

cap.release()
cv.destroyAllWindows()
