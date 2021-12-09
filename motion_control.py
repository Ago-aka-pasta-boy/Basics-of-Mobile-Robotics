import math

KP = 75
EPSILON = 0.1  # 15 pixels
NORMAL_SPEED = 150
MAX_DIV = 1

POSITION_ERROR = 20



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
    #motor_left = NORMAL_SPEED/max(1,abs(err)) - KP * err
    #motor_right = NORMAL_SPEED/max(1,abs(err)) + KP * err
    motor_left = NORMAL_SPEED/2
    motor_right = NORMAL_SPEED/4
    return motor_left, motor_right


def get_error(robot_pos, next_goal):
    alpha = robot_pos[1]

    dy = robot_pos[0][1]-next_goal[1]
    dx = next_goal[0]-robot_pos[0][0]
    beta = math.atan2(dy, dx)

    err = beta-alpha
    """
    if err > math.pi:
        err = err -2 * math.pi
    if err < -math.pi:
        err = err + 2 * math.pi
    """
    return err


def check_robot_arrived(robot_pos, next_goal):
    if math.dist(robot_pos[0], next_goal) < POSITION_ERROR:
        print("eureka en grand")
        return True

    else:
        return False


