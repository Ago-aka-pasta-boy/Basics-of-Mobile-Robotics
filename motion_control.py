import math
 
KP = 75
EPSILON = 0.1  # 15 pixels
NORMAL_SPEED = 300
MAX_DIV = 1
OBST_THR_L = 10      # low obstacle threshold 
OBST_THR_H = 60      # high obstacle threshold 


POSITION_ERROR = 50

def check_obstacle(prox_sensors):
    # acquisition from the proximity sensors to detect obstacles
    obst = [prox_sensors[0], prox_sensors[1], \
            prox_sensors[2], prox_sensors[3], \
            prox_sensors[4]]
    mean_obst_left = (obst[0] + obst[1])//2
    mean_obst_right = (obst[3] + obst[4])//2
    obst_front = obst[2]
    
    if (mean_obst_right > OBST_THR_H) or (mean_obst_left > OBST_THR_H) or (obst_front > OBST_THR_H) :
        obstacle = True
    else:
        obstacle = False
    return obstacle, obst_front, mean_obst_left, mean_obst_right


def speed_control(err):
    """
    if err < -EPSILON:
        # turn right
        motor_left = NORMAL_SPEED + KP*abs(err)
        motor_right = NORMAL_SPEED - KP*abs(err)
        print("first if")
#/(max(abs(err),MAX_DIV))
    elif err > EPSILON: 
    # turn left     
        motor_left = NORMAL_SPEED - KP * abs(err)
        motor_right = NORMAL_SPEED + KP * abs(err)
        print("second if")

    else:
        # go straight
        motor_left = NORMAL_SPEED
        motor_right = NORMAL_SPEED
        """
    motor_left = NORMAL_SPEED/max(1,10*abs(err)) - KP * err
    motor_right = NORMAL_SPEED/max(1,10*abs(err)) + KP * err
    
    
    sgn = lambda x: x/max(abs(x),1e-12)
    
    motor_left = sgn(motor_left)*min(abs(motor_left),300)
    motor_right = sgn(motor_right)*min(abs(motor_right),300)
    
    return motor_left, motor_right


def get_error(robot_pos, next_goal):
    alpha = robot_pos[1]

    dy = robot_pos[0][1]-next_goal[1]
    dx = next_goal[0]-robot_pos[0][0]
    beta = math.atan2(dy, dx)

    err = beta-alpha
    err = (err+math.pi)%(2*math.pi) - math.pi
    
    return err


def check_robot_arrived(robot_pos, next_goal):
    if math.dist(robot_pos[0], next_goal) < POSITION_ERROR:
        print("eureka en grand")
        return True

    else:
        return False


