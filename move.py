from djitellopy import Tello
import time
import numpy as np

PIXLES_X = 500
PIXLES_Y = 500
MAX_SECONDS = 1

directions = ['r', 'l', 'u', 'd', 'f', 'b']


def velocity_up(x):
    tello.send_rc_control(0, 0, 50, 0)
    time.sleep(MAX_SECONDS / PIXLES_X * np.abs(x))
    tello.send_rc_control(0, 0, 0, 0)


def velocity_up_down(x):
    tello.send_rc_control(0, 0, int(np.sign(x)*50), 0)
    time.sleep(MAX_SECONDS / PIXLES_X * np.abs(x))
    tello.send_rc_control(0, 0, 0, 0)


def velocity_left_right(x):
    tello.send_rc_control(int(np.sign(x)*50), 0, 0, 0)
    time.sleep(MAX_SECONDS / PIXLES_X * np.abs(x))
    tello.send_rc_control(0, 0, 0, 0)


def velocity_forward_backward(x):
    tello.send_rc_control(0, int(np.sign(x)*50), 0, 0)
    time.sleep(MAX_SECONDS / PIXLES_X * np.abs(x))
    tello.send_rc_control(0, 0, 0, 0)

if __name__ == '__main__':
    tello = Tello()
    tello.connect()
    print(str(tello.get_battery()) + "%")

    tello.takeoff()
    time.sleep(1)

    while True:
        time.sleep(1)
        up_down = 0
        left_right = 0
        forward_backward = 0
        tello.send_rc_control(left_right, forward_backward, up_down,0)
        # x = int(input("coordinate: "))
        d = input("direction: ")
        if d not in directions:
            break
        if d == 'l':
            left_right = -50
        if d == 'r':
            left_right = 50
        if d == 'u':
            up_down = 50
        if d == 'd':
            up_down = -50
        if d == 'f':
            forward_backward = 50
        if d == 'b':
            forward_backward = -50
        tello.send_rc_control(left_right, forward_backward, up_down,0)
        # if np.abs(x)<PIXLES_X/10:
        #     tello.send_rc_control(int(-1*np.sign(x)*20), 0, 0, 0)
        #     time.sleep(0.2)
        #     tello.send_rc_control(0,0,0,0)

    tello.land()