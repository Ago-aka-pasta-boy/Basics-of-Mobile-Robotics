import math
 
KP = 75              #proportional gain for the P-controller
NORMAL_SPEED = 300   #normal speed of the robot
OBST_THR_L = 10      # low obstacle threshold 
OBST_THR_H = 60      # high obstacle threshold 
POSITION_ERROR = 50 

def check_obstacle(prox_sensors):
    """"
    --- 
    Description : Checks if there is an obstacle somewhere in front of the robot
    ---
    Input: Values of the proximity sensors measured via Thymio
    
    ---
    Outputs: - obstacle: a boolean  True = there is an obstacle somewhere in 
                                            front of the robot
                                    False = no obstacle in front of the robot
             - obst_front: value of the third proximity sensor
             - mean_obst_left: mean value between the first and the second 
                               proximity sensors
             - mean_obst_right: mean value between the fourth and the fifth 
                                proximity sensors
    """
    
    # acquisition from the proximity sensors to detect obstacles
    obst = [prox_sensors[0], prox_sensors[1], \
            prox_sensors[2], prox_sensors[3], \
            prox_sensors[4]]
    
    mean_obst_left = (obst[0] + obst[1])//2
    mean_obst_right = (obst[3] + obst[4])//2
    obst_front = obst[2]
    
    #determines if there is an obstacle or not in front of the robot
    if (mean_obst_right > OBST_THR_H) or (mean_obst_left > OBST_THR_H) \
        or (obst_front > OBST_THR_H) :
        obstacle = True
    else:
        obstacle = False
    return obstacle, obst_front, mean_obst_left, mean_obst_right


def speed_control(err):
    """
    ---
    Description : Sets the speed of the robot's wheels depending on the 
                 error value
    ---
    Input: Angle error between the robot's position and its next goal position
        
    ---
    Outputs: - motor_left: Value to assign to the left wheel of the robot
             - motor_right: Value to assign to the right wheel of the robot
    """
    
    motor_left = NORMAL_SPEED/max(1,10*abs(err)) - KP*err
    motor_right = NORMAL_SPEED/max(1,10*abs(err)) + KP*err
    
    
    sgn = lambda x: x/max(abs(x),1e-12)
    
    motor_left = sgn(motor_left)*min(abs(motor_left),300)
    motor_right = sgn(motor_right)*min(abs(motor_right),300)
    
    return motor_left, motor_right


def get_error(robot_pos, next_goal):
    """
    --- 
    Description: Calculates the error between the angle error 
                  between the robot's position and its next goal position 
    
    ---
    Inputs: - robot_pos: Value of the robot position ((x, y), angle)
            - next_goal: Value of the next goal position
    
    ---
    Output: Angle error between the robot's position and its next goal position
        
    """
   
    alpha = robot_pos[1]

    dy = robot_pos[0][1] - next_goal[1]
    dx = next_goal[0] - robot_pos[0][0]
    beta = math.atan2(dy, dx)

    err = beta - alpha
    err = (err + math.pi)%(2*math.pi) - math.pi
    
    return err


def check_robot_arrived(robot_pos, next_goal):
    """
    --- 
    Description: Checks if the robot arrived to its goal position
    
    ---
    Inputs: - robot_pos: Value of the robot position ((x, y), angle)
            - next_goal: Value of the next goal position
    
    ---
    Output: - True: The robot arrived to its goal
            - False: The robot did not arrived to its goal
        
    """
    
    if math.dist(robot_pos[0], next_goal) < POSITION_ERROR:
        print("eureka en grand")
        return True

    else:
        return False


