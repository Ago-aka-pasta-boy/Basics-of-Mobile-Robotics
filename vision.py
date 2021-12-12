import numpy as np
import cv2 as cv
import math
import copy

APPROX_FACTOR = 0.03
EPSILON = 0.01
TEXT_OFFSET = 5
FULL = -1
ALL_CTRS = -1
KERNEL_SIZE = (5, 5)
FONT_SCALE = 1
SCALING_FACTOR = 50
ROBOT_LINE = 50
THRESH = 100

MAX_CTR_SIZE = 1000
MIN_CTR_SIZE = 200

ROBOT_RADIUS = 10
GOAL_RADIUS = 15

MAX_VERT_OBST = 8
MIN_VERT_OBST = 2

PIXEL_WIDTH = 3
RED_CHANNEL = 0

GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)

#%%
def extract_obstacles(img):
    """"
    --- 
    Description: Finds all the black obstacles and extract their vertices 
                from the image
    ---
    Input: - img: Image captured by the camera
    
    ---
    Outputs: - obstacles: [array([[[v1_1x, v1_1y]],[[v1_2x, v1_2y]],...,
                                  [[v1_Nx, v1_Ny]]], dtype = int32),
                           array([[[v2_1x, v2_1y]],[[v2_2x, v2_2y]],...,
                                  [[v2_Nx, v2_Ny]]], dtype = int32),
                           .....,
                           array([[[vM_1x, vM_1y]],[[vM_2x, vM_2y]],...,
                                  [[vM_Nx, vM_Ny]]], dtype = int32),]
                           where vM_Nx is the x coordinate of the Nth vertex of 
                           the Mth obstacle
             - Bolean returns false: if no obstacles found
    """
    img_red = img[:, :, RED_CHANNEL]
    blurred = cv.GaussianBlur(img_red, KERNEL_SIZE, cv.BORDER_DEFAULT)
    img_filtered = cv.morphologyEx(blurred, cv.MORPH_OPEN,np.ones(KERNEL_SIZE))
    ret, img_ready = cv.threshold(img_filtered, THRESH, 255, cv.THRESH_BINARY)
    cv.imshow("Thresholded", img_ready)
    contours, hierarchy = cv.findContours(img_ready, cv.RETR_LIST, \
                                          cv.CHAIN_APPROX_SIMPLE)
    im2 = np.zeros(img.shape)
    approximations = []
    
    for count in range(len(contours)):
        epsilon = APPROX_FACTOR*cv.arcLength(contours[count], True)
        if (cv.arcLength(contours[count], True) < MAX_CTR_SIZE) & \
            (cv.arcLength(contours[count], True) > MIN_CTR_SIZE):
            approximations.append(cv.approxPolyDP(contours[count], epsilon, \
                                                  closed=True))
    cv.drawContours(im2, approximations, ALL_CTRS, GREEN, PIXEL_WIDTH)
    cv.imshow("Contours", im2)
    obstacles = []
    
    for i in range(len(approximations)):
        if (len(approximations[i]) < MAX_VERT_OBST) and \
            (len(approximations[i]) > MIN_VERT_OBST):
            obstacles.append(approximations[i])
    im3 = np.zeros(img.shape)
    cv.drawContours(im3, obstacles, ALL_CTRS, GREEN, PIXEL_WIDTH)
    cv.imshow("obstacles", im3)
    
    if not obstacles:
        return False, []
    return True, obstacles

#%%
def expand_obstacles(obstacles):
    """"
    --- 
    Description: Expands the size of the obstacles to take the robot size in 
                account for the path
    ---
    Input: - obstacles: [array([[[v1_1x, v1_1y]],[[v1_2x, v1_2y]],...,
                                  [[v1_Nx, v1_Ny]]], dtype = int32),
                        array([[[v2_1x, v2_1y]],[[v2_2x, v2_2y]],...,
                               [[v2_Nx, v2_Ny]]], dtype = int32),
                        .....,
                        array([[[vM_1x, vM_1y]],[[vM_2x, vM_2y]],...,
                               [[vM_Nx, vM_Ny]]], dtype = int32),]
                        where vM_Nx is the x coordinate of the Nth vertex of 
                        the Mth obstacle
    
    ---
    Output: - ex_obstacles: same format as obstacles but with expanded 
                             obstacles 
    """
    ex_obstacles = copy.deepcopy(obstacles)
    
    for c in range(len(obstacles)):
        moments = cv.moments(obstacles[c])
        
        if not moments["m00"]:
            moments["m00"] = EPSILON
        if not moments["m00"]:
            moments["m00"] = EPSILON
            
        cx = int(moments["m10"]/moments["m00"])
        cy = int(moments["m01"]/moments["m00"])
        
        for d in range(len(obstacles[c])):
            vec_x = obstacles[c][d][0][0] - cx
            vec_y = obstacles[c][d][0][1] - cy
            vec = np.array([vec_x, vec_y])
            vec = vec/np.linalg.norm(vec)
            ex_obstacles[c][d][0][0] = obstacles[c][d][0][0] \
                                        + vec[0]*SCALING_FACTOR
            ex_obstacles[c][d][0][1] = obstacles[c][d][0][1] \
                                        + vec[1]*SCALING_FACTOR
    return ex_obstacles

#%%
def annotate_path(path, img):
    """"
    --- 
    Description:  Draws a line from start to end on the provided image
    ---
    Inputs: - img: Canvas image to draw the path on
            - path: List of coordinates of points along the path.
                   Example: [(10,15), (20,30),..., (50,50)]
                   where start = (10,15) and goal = (50,50) 
    """
    for vert in range(len(path) - 1):
        cv.line(img, path[vert], path[vert + 1], WHITE, PIXEL_WIDTH)

#%%
def annotate_robot(robot_pos, img):
    """"
    --- 
    Description:  Draws the robot on the provided image
    ---
    Inputs: - img: Canvas image to draw the robot on
            - robot_pos:  [(x, y), angle] Value of the robot position
            
    """
    cv.circle(img, robot_pos[0], ROBOT_RADIUS, RED, FULL)
    point2 = (int(robot_pos[0][0] + math.cos(robot_pos[1])*ROBOT_LINE), \
              int(robot_pos[0][1] - math.sin(robot_pos[1])*ROBOT_LINE))
    cv.line(img, robot_pos[0], point2, RED, PIXEL_WIDTH)
    cv.putText(img, "Thymio", robot_pos[0], cv.FONT_HERSHEY_SIMPLEX, \
               FONT_SCALE, BLUE, PIXEL_WIDTH)
        

#%%
def annotate_goal(goal_pos, img):
    """"
    --- 
    Description:  Draws the goal on the provided 
    
    ---
    Inputs:
        - goal_pos: (x, y) Value of the goal position
        - img: Canvas image to draw the goal on
    
    """
    cv.circle(img, (goal_pos[0], goal_pos[1]), GOAL_RADIUS, GREEN, FULL)
    cv.putText(img, "Goal", \
               (goal_pos[0] + TEXT_OFFSET, goal_pos[1] + TEXT_OFFSET), \
                cv.FONT_HERSHEY_SIMPLEX, FONT_SCALE, GREEN, PIXEL_WIDTH)

#%%
def annotate_arch(arch_pos, img):
    """"
    --- 
    Description: Draws the way through the arch on the provided image
    
    ---
    Inputs: 
        - arch_pos: ((x1, y1), (x2, y2)) Value of the two points before 
                    and after the arch
        - img: Canvas image to draw the arch on
     
    """
   
    point1 = arch_pos[len(arch_pos) - 2]
    point2 = arch_pos[len(arch_pos) - 1]
    cv.line(img, point1, point2, RED, PIXEL_WIDTH)
    cv.putText(img, "arch", point1, cv.FONT_HERSHEY_SIMPLEX, FONT_SCALE, RED, \
               PIXEL_WIDTH)

#%%
def convert_vertice(obstacles):
    """"
    --- 
    Description: Converts the base listing of opencv for contours into an array 
                 of arrays of tuples used in global navigation
    ---
    Input: - obstacles: [array([[[v1_1x, v1_1y]],[[v1_2x, v1_2y]],...,
                                  [[v1_Nx, v1_Ny]]], dtype = int32),
                        array([[[v2_1x, v2_1y]],[[v2_2x, v2_2y]],...,
                               [[v2_Nx, v2_Ny]]], dtype = int32),
                        .....,
                        array([[[vM_1x, vM_1y]],[[vM_2x, vM_2y]],...,
                               [[vM_Nx, vM_Ny]]], dtype = int32),]
                        where vM_Nx is the x coordinate of the Nth vertex of 
                        the Mth obstacle
    
    ---
    Outputs: - converted_vertices : [[v1.1, v1.2,..., v1.N1], 
                                     [v2.1,..., v2.N2],..., 
                                     [vO.1,..., vO.NO]]
                where O is the total number of obstacles,
                N1,N2,...,NO is the number of vertices for obstacles 1,2,...,O
                vk.1, vk.2,...,vk.Nk are tuples (x,y) with coordinates of k-th 
                obstacle's vertices
    """
    
    converted_vertices = []
    
    for c in range(np.size(obstacles, 0)):
        conv = []
        
        for d in range(len(obstacles[c])):
            conv.append(tuple(obstacles[c][d][0]))
        converted_vertices.append(conv)
        
    return converted_vertices

