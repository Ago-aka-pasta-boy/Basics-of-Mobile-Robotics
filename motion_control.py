import math

KP = 150
EPSILON = 0.1  # 15 pixels
NORMAL_SPEED = 150
MAX_DIV = 1


def speed_control(err):
    if err < -EPSILON:
        # turn right
        motor_left = NORMAL_SPEED/(max(abs(err),MAX_DIV)) + KP*err
        motor_right = NORMAL_SPEED/(max(abs(err),MAX_DIV)) - KP*err

    elif err > EPSILON:
        # turn left
        motor_left = NORMAL_SPEED/(max(abs(err),MAX_DIV)) - KP * err
        motor_right = NORMAL_SPEED/(max(abs(err),MAX_DIV)) + KP * err

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



