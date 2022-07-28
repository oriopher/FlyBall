import numpy as np
from image_3d import Image3D
from djitellopy import Tello
from borders import Borders
from utils import FLOOR_HEIGHT, DRONE_DEFAULT_HEIGHT

def track_3d(image_3d: Image3D, tello: Tello, dest_x: float, dest_y: float, dest_z: float):
    x_cm_rel = dest_x - image_3d.get_phys_drone(0)
    y_cm_rel = dest_y - image_3d.get_phys_drone(1)
    z_cm_rel = dest_z - image_3d.get_phys_drone(2)

    left_right, for_back, up_down = 0, 0, 0
    left_right = lin_velocity_with_two_params(x_cm_rel, image_3d.velocity_x_drone, 'x')
    for_back = lin_velocity_with_two_params(y_cm_rel, image_3d.velocity_y_drone, 'y')
    up_down = lin_velocity_with_two_params(z_cm_rel, image_3d.velocity_z_drone, 'z')
    # left_right = lin_velocity_with_control(x_cm_rel, image_3d.velocity_x_drone, 'x')
    # for_back = lin_velocity_with_control(y_cm_rel, image_3d.velocity_y_drone, 'y')
    # up_down = lin_velocity_z(z_cm_rel)
    if tello.send_rc_control:
        tello.send_rc_control(left_right, for_back, up_down, 0)


def track_balloon(image_3d: Image3D, tello: Tello):
    dest_x = image_3d.get_phys_balloon(0)
    dest_y = image_3d.get_phys_balloon(1)
    track_2d(image_3d, tello, dest_x, dest_y)


def seek_middle(image_3d: Image3D, tello: Tello, borders: Borders):
    dest_x = borders.x_middle
    dest_y = borders.y_middle
    track_2d(image_3d, tello, dest_x, dest_y)


def track_2d(image_3d: Image3D, tello: Tello, dest_x: int, dest_y: int):
    track_3d(image_3d, tello, dest_x, dest_y, DRONE_DEFAULT_HEIGHT)


def lin_velocity_with_acc(cm_rel, real_vel):
    # this is an old function
    # this function assumes the drone is looking at the cameras.
    a = 1.5
    b = 1
    stoping_limit = 5
    max_vel = 30

    velocity_pot = int(min(abs(a * cm_rel), max_vel))

    # ball is in left side of the drone and it's not too fast
    if cm_rel > stoping_limit and cm_rel < b * abs(
            real_vel) and real_vel < 0:  # If the velocity is positive we would like to stop
        velocity = -velocity_pot

    # ball is in right side of the drone and it's not too fast
    elif cm_rel < -stoping_limit and abs(cm_rel) < b * abs(
            real_vel) and real_vel > 0:  # If the velocity is negative we would like to stop
        velocity = velocity_pot

    else:
        velocity = 0

    return velocity


def lin_velocity_with_two_params(cm_rel, real_velocity, direction):
    # this function assumes the drone is looking at the cameras.

    # These parameters acheive a hit within 2-3 seconds using lin + sqrt. 
    # there is a plot of this function in the documentation.
    # MIN_VEL = 9 # under this speed the tello recieves this as 0
    # LOWER_BOUND = 5 # when the drone is closer than this we will just let it stop
    # if direction == 'z':
    #     MAX_VEL = 30
    #     A_SQRT = 2
    #     A_LINEAR = 0.9
    #     B = 1.2
    #     STOPPING_VEL = 0
    #     C = 0
    # elif direction == 'x' or direction == 'y':
    #     STOPPING_VEL = 20
    #     MAX_VEL = 60
    #     A_SQRT = 3
    #     A_LINEAR = 1
    #     B = 0.7
    #     C = 5
    # else:
    #     return 0

    MIN_VEL = 9 # under this speed the tello recieves this as 0
    LOWER_BOUND = 5 # when the drone is closer than this we will just let it stop

    STOPPING_VEL = 30 # The velocity applied in order to stop faster
    MAX_VEL = 70
    A_SQRT = 3
    A_LINEAR = 1
    C = 5
    B = 0.6 # Associated with the stopping distance

    if direction == 'z':
        STOPPING_VEL = 0
        B = 0.3
        A_LINEAR = 1.5
        LOWER_BOUND = 2

    limit = max(B * abs(real_velocity), LOWER_BOUND)

    if abs(cm_rel) < limit:
        if abs(cm_rel) < LOWER_BOUND:
            velocity = 0
        else:
            velocity = np.sign(real_velocity) * STOPPING_VEL

    else:
        # velocity_pot = int(min(A * (abs(cm_rel) - limit) + MIN_VEL, MAX_VEL))
        velocity_pot = int(min(max(A_SQRT * np.sqrt(abs(cm_rel) - limit), A_LINEAR * (abs(cm_rel) - limit) - C) + MIN_VEL, MAX_VEL))
        velocity = -np.sign(cm_rel) * velocity_pot

    if direction == 'x':
        return int(velocity)
    elif direction == 'y':
        return int(velocity)
    elif direction == 'z':
        return -int(velocity)


def lin_velocity_z(cm_rel):
    # this function assumes the drone is looking at the cameras.
    MAX_VEL = 20
    A = 1.4

    velocity_pot = int(min(A * (abs(cm_rel)), MAX_VEL))
    velocity = np.sign(cm_rel) * velocity_pot

    return int(velocity)


def lin_velocity_with_control(cm_rel, real_velocity, direction):
    # this function assumes the drone is looking at the cameras.
    MAX_VEL = 80
    UPPER_LIMIT = 20
    LOWER_LIMIT = 5
    VELOCITY_LIMIT = 20
    STOPPING_VEL = 20
    A_UPPER = 1.5
    A_LOWER = 0.7

    velocity_pot_upper = int(min(A_UPPER * (abs(cm_rel) - LOWER_LIMIT), MAX_VEL))
    velocity_pot_lower = int(min(A_LOWER * (abs(cm_rel) - LOWER_LIMIT), MAX_VEL))

    # if drone is too fast, stop earlier
    if UPPER_LIMIT > abs(cm_rel) > LOWER_LIMIT and abs(real_velocity) > VELOCITY_LIMIT:
        velocity = 0

    # drone is not too fast, continue in original speed
    elif UPPER_LIMIT > abs(cm_rel) > LOWER_LIMIT:
        # velocity = np.sign(real_velocity)*velocity_pot_lower
        velocity = -np.sign(cm_rel) * velocity_pot_lower

    # if drone is too fast and close to the baloon, set negative velocity
    elif abs(cm_rel) < LOWER_LIMIT and abs(real_velocity) > VELOCITY_LIMIT:
        velocity = -np.sign(real_velocity) * STOPPING_VEL

    elif abs(cm_rel) > UPPER_LIMIT:
        velocity = np.sign(cm_rel) * velocity_pot_upper

    # if we got here, drone is close to the baloon with low speed
    else:
        velocity = 0

    velocity = int(velocity)
    if direction == 'x':
        return -velocity
    elif direction == 'y':
        return -velocity
