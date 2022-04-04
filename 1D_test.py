import numpy as np
import cv2
from djitellopy import Tello
import time

THRESHOLD_SIZE = 7

RED_LOWER_BOUNDS = (136, 87, 111)
RED_UPPER_BOUNDS = (180, 255, 255)
GREEN_LOWER_BOUNDS = (25, 52, 72)
GREEN_UPPER_BOUNDS = (102, 255, 255)
BLUE_LOWER_BOUNDS = (94, 80, 2)
BLUE_UPPER_BOUNDS = (120, 255, 255)
PINK_LOWER_BOUNDS = (136, 87, 111)
PINK_UPPER_BOUNDS = (180, 255, 255)
NO_LOWER_BOUNDS = (0, 0, 0)
NO_UPPER_BOUNDS = (255, 255, 255)


def capture_video(tello):
    vid = cv2.VideoCapture(0)

    while True:
        # Capture the video frame by frame
        ret, frame = vid.read()

        # recognize ball
        x_ball, y_ball = find_object_coordinates(frame, PINK_LOWER_BOUNDS, PINK_UPPER_BOUNDS)
        if x_ball != 0 and y_ball != 0:
            cv2.circle(frame, (int(x_ball), int(y_ball)), 15, (0, 0, 100), 3)

        # recognize drone
        x_drone, y_drone = find_object_coordinates(frame, GREEN_LOWER_BOUNDS, GREEN_UPPER_BOUNDS)
        if x_drone != 0 and y_drone != 0:
            cv2.circle(frame, (int(x_drone), int(y_drone)), 15, (0, 0, 100), 3)

        # Display the resulting frame
        cv2.imshow('frame', frame)

        # move the drone towards the ball
        if x_drone + 150 < x_ball:  # 150 pixels for safety
            trackBall_1D(tello, 1)
        elif x_drone > x_ball + 150:
            trackBall_1D(tello, 2)
        else:
            trackBall_1D(tello, 0)

        # the 'q' button is set as the quitting button
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


def find_object_coordinates(img, lower_bound, upper_bound):
    # convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # define kernel size
    kernel = np.ones((THRESHOLD_SIZE, THRESHOLD_SIZE),np.uint8)
    # Remove unnecessary noise from mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # target = cv2.bitwise_and(img, img, mask = mask)
    # cv2.imshow('target', target)
    # cv2.waitKey(0)

    balloon_pixels = np.argwhere(mask)
    if len(balloon_pixels) == 0:
        return 0, 0
    x_coor = np.median(balloon_pixels[:,1])
    y_coor = np.median(balloon_pixels[:,0])

    return x_coor, y_coor


def trackBall_1D(tello, dir):
    if dir == 1:  # ball is in left side of the drone (computer camera is opposite)
        left_right = 60
    elif dir == 2:  # ball is in right side of the drone
        left_right = -60
    else:  # drone is under the ball
        left_right = 0
        for_back = 0
        up_down = 0
    if tello.send_rc_control:
        tello.send_rc_control(left_right, for_back, up_down, 0)


if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    tello.takeoff()
    time.sleep(8)
    capture_video(tello)
