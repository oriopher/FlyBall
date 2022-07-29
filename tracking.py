import numpy as np
from image_3d import Image3D
from djitellopy import Tello
from borders import Borders
from common import DRONE_DEFAULT_HEIGHT, reachability
from prediction import NumericBallPredictor

def track_3d(image_3d: Image3D, tello: Tello, dest_x: float, dest_y: float, dest_z: float):
    x_cm_rel = dest_x - image_3d.get_phys_drone(0)
    y_cm_rel = dest_y - image_3d.get_phys_drone(1)
    z_cm_rel = dest_z - image_3d.get_phys_drone(2)

    left_right, for_back, up_down = 0, 0, 0
    left_right = velocity_control_function(x_cm_rel, image_3d.velocity_x_drone, 'x')
    for_back = velocity_control_function(y_cm_rel, image_3d.velocity_y_drone, 'y')
    up_down = velocity_control_function(z_cm_rel, image_3d.velocity_z_drone, 'z')
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



def velocity_control_function(cm_rel, real_velocity, direction):
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


def track_searching(image_3d: Image3D, tello: Tello, dest_x: float, dest_y: float, go_up = False):
    pass

def track_hitting(image_3d: Image3D, tello: Tello, dest_x : float, dest_y : float, dest_z : float):
    rx = dest_x - image_3d.get_phys_drone(0)
    ry = dest_y - image_3d.get_phys_drone(1)
    rz = dest_z - image_3d.get_phys_drone(2)
    r = np.sqrt(rx**2 + ry**2 + rz**2)
    theta = np.arccos(rz/r)
    phi = np.arctan2(ry,rx)
    vz = 100
    vx = int(vz*np.tan(theta)*np.cos(phi))
    vy = int(vz*np.tan(theta)*np.sin(phi))

    left_right, for_back, up_down = vx, vy, vz
    if tello.send_rc_control:
        tello.send_rc_control(left_right, for_back, up_down, 0)