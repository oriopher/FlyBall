import numpy as np
from image_3d import Image3D
from djitellopy import Tello
from borders import Borders

FLOOR_HEIGHT = -60
DRONE_DEFAULT_HEIGHT = FLOOR_HEIGHT + 80


def track_3d(image_3d: Image3D, tello: Tello, dest_x: float, dest_y: float, dest_z: float):
    x_cm_rel = dest_x - image_3d.phys_drone_median[0]
    y_cm_rel = dest_y - image_3d.phys_drone_median[1]
    z_cm_rel = dest_z - image_3d.phys_drone_median[2]

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
    dest_x = image_3d.get_phys_mean_balloon(0)
    dest_y = image_3d.get_phys_mean_balloon(1)
    dest_z = DRONE_DEFAULT_HEIGHT

    track_3d(image_3d, tello, dest_x, dest_y, dest_z)


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
    # C = 2
    # VELOCITY_LIMIT = 20
    # STOPPING_VEL = 0
    # MAX_VEL = 40
    # A = 1.5
    # B = 1
    # VELOCITY_LIMIT = 10
    # STOPPING_VEL = 20

    MAX_VEL = 30
    if direction == 'z':
        A = 1.5
        B = 1.2
    else:
        A = 1.5
        B = 1.5

    

    limit = B * abs(real_velocity)
    velocity_pot = int(min(A * (abs(cm_rel) - limit), MAX_VEL))

    if abs(cm_rel) < limit:
        velocity = 0
        # if abs(real_velocity) > VELOCITY_LIMIT:
        #     velocity = -np.sign(real_velocity) * STOPPING_VEL
        #     # velocity = -np.sign(real_velocity)*min(abs(real_velocity), MAX_VEL)             

    else:
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
