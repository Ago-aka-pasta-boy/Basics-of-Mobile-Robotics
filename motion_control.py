import positions as pos
import math

KP = 2
EPSILON = 15  # 15 pixels
NORMAL_SPEED = 150
POSITION_ERROR = 5

def speed_control(err):
    if err < -EPSILON:
        # turn right
        motor_left_target = KP*err
        motor_right_target = -KP*err

    elif err > EPSILON:
        # turn left
        motor_left_target = -KP * err
        motor_right_target = KP * err

    else:
        # go straight
        motor_left_target = NORMAL_SPEED
        motor_right_target = NORMAL_SPEED

    return motor_left_target, motor_right_target


def get_error(img, next_goal):
    robot_pos = pos.get_robot_position(img)

    alpha = robot_pos[1,0]

    dy = robot_pos[0,1]-next_goal[1]
    dx = next_goal[0]-robot_pos[0,0]
    beta = math.atan2(dy, dx)

    err = beta-alpha

    return err


def check_robot_arrived(robot_pos, next_goal):
    if math.dist(robot_pos[0], next_goal) < POSITION_ERROR:
        return True

    else:
        return False


