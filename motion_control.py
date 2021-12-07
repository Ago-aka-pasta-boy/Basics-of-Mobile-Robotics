import math

KP = 10
EPSILON = 0.1  # 15 pixels
NORMAL_SPEED = 150
POSITION_ERROR = 5

def speed_control(err):
    if err < -EPSILON:
        # turn right
        motor_left = NORMAL_SPEED/err + KP*err
        motor_right = NORMAL_SPEED/err - KP*err

    elif err > EPSILON:
        # turn left
        motor_left = NORMAL_SPEED/err - KP * err
        motor_right = NORMAL_SPEED/err + KP * err

    else:
        # go straight
        motor_left = NORMAL_SPEED
        motor_right = NORMAL_SPEED

    return motor_left, motor_right


def get_error(robot_pos, next_goal):
    alpha = robot_pos[1]

    dy = robot_pos[0][1]-next_goal[1]
    dx = next_goal[0]-robot_pos[0][0]
    beta = math.atan2(dy, dx)

    err = beta-alpha

    return err


def check_robot_arrived(robot_pos, next_goal):
    if math.dist(robot_pos[0], next_goal) < POSITION_ERROR:
        return True

    else:
        return False


