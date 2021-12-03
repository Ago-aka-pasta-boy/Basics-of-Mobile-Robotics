
import tdmclient
await 
speed0 = 100       # nominal speed
speedGain = 2      # gain used with ground gradient
obstThrL = 10      # low obstacle threshold to switch state 1->0
obstThrH = 20      # high obstacle threshold to switch state 0->1
obstSpeedGain = 5  # /100 (actual gain: 5/100=0.05)

state = 0          # 0= go to goal, 1=obstacle avoidance
diffDelta = 0          # difference between next goal and actual position
obst = [0,0]            # measurements from left and right prox sensors

timer_period[0] = 10   # 10ms sampling time

coordx = 0      #define to access to the x coordinate in a list
coordy = 1      #define to access to the y coordinate in a list
proxsens1 = 0   #proximity sensor 1
proxsens5 = 4   #proximity sensor 2
statego2goal = 0
stateobstavoid = 1

#test
#how to deal with lists of lists in Thymio ???
list_vertices = [[(10,10),(20,10),(15,18.66)],\
                  [(30,25),(40,15),(53.66,18.66),(57.32,32.32),(47.32,42.32),(33.66,38.66)],\
                      [(59.27,27.55),(54.63,70.85),(67.09,69.57),(73.97,36.5)]]
start = (0,0)
goal = (75.88,42.09)
list_neighbours = global_path.find_all_paths(list_vertices, start, goal)
names_path = shortest_path.find_shortest_path(list_vertices, list_neighbours, start, goal)


steps_to_goal = shortest_path.names_to_subpaths(names_path, obstacle_vertices, start, goal)
next_goal = []
actual_pos = (0,0,0) #thymio_coords ? #np.array avce x, y, angle entre -pi et pi

@onevent 
def timer0():
    global prox_ground_delta, prox_horizontal, motor_left_target, motor_right_target, state, obst, obstThrH, obstThrL, obstSpeedGain, speed0, speedGain 
    
    for i in steps_to_goal:
        if actual_pos[coordx] >= steps_to_goal[coordx][i] \
        and actual_pos[coordy] >= steps_to_goal[coordx][i]:
        
            next_goal[i] = steps_to_goal[coordy][i]
            diffDelta = next_goal[i] - actual_pos
        end
    
        # acquisition from the proximity sensors to detect obstacles
        obst = [prox_horizontal[proxsens1], prox_horizontal[proxsens5]]
        
        # tdmclient does not support yet multiple and/or in if statements:
        if state == statego2goal: 
            # switch from goal tracking to obst avoidance if obstacle detected
            if (obst[0] > obstThrH):
                state = stateobstavoid
            elif (obst[1] > obstThrH):
                state = stateobstavoid
        elif state == stateobstavoid:
            if obst[0] < obstThrL:
                if obst[1] < obstThrL : 
                    # switch from obst avoidance to goal tracking if obstacle got unseen
                    state = statego2goal
        if  state == statego2goal :
            # goal tracking: turn toward the goal
            leds_top = [0,0,0]
            motor_left_target = speed0 - speedGain * diffDelta
            motor_right_target = speed0 + speedGain * diffDelta
        else:
            leds_top = [30,30,30]
            # obstacle avoidance: accelerate wheel near obstacle
            motor_left_target = speed0 + obstSpeedGain * (obst[0] // 100)
            motor_right_target = speed0 + obstSpeedGain * (obst[1] // 100)