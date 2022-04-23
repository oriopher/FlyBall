from pickle import FALSE, TRUE
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
PINK_LOWER_BOUNDS = (120, 60, 111)
PINK_UPPER_BOUNDS = (180, 255, 255)
YELLOW_LOWER_BOUNDS = (20, 70, 70)
YELLOW_UPPER_BOUNDS = (80, 255, 255)
NO_LOWER_BOUNDS = (0, 0, 0)
NO_UPPER_BOUNDS = (255, 255, 255)


def play_ball(tello):
    vid = cv2.VideoCapture(0)
    tello.connect()
    tookoff = False
    started = False


    while True:
        # Capture the video frame by frame
        ret, frame = vid.read()

        # recognize ball
        x_ball, y_ball = find_object_coordinates(frame, PINK_LOWER_BOUNDS, PINK_UPPER_BOUNDS)
        if x_ball != 0 and y_ball != 0:
            cv2.circle(frame, (int(x_ball), int(y_ball)), 15, (0, 100, 0), 3)

        # recognize drone
        x_drone, y_drone = find_object_coordinates(frame, YELLOW_LOWER_BOUNDS, YELLOW_UPPER_BOUNDS) 

        if x_drone != 0 and y_drone != 0:
            cv2.circle(frame, (int(x_drone), int(y_drone)), 15, (0, 0, 100), 3)


        if tookoff == True:

            if started == False:
                left_limit = 300
                right_limit = 400
            else:
                left_limit = 150
                right_limit = 480

            cv2.circle(frame, (int(left_limit), int(200)), 15, (100, 0, 0), 3)
            cv2.circle(frame, (int(right_limit), int(200)), 15, (100, 0, 0), 3)
        
            if x_drone > right_limit or x_drone < left_limit: # emergency stop the drone is getting out of the frame.
                print("where are you going?")
                x_drone, y_drone = find_object_coordinates(frame, BLUE_LOWER_BOUNDS, BLUE_UPPER_BOUNDS)

                if x_drone > right_limit or x_drone < left_limit: # making sure it is correct
                    print("Stay here!")
                    break
            if started == True:    
                # move the drone towards the ball
                trackBall_1D(tello, x_drone, y_drone, x_ball, y_ball)

        # Display the resulting frame
        cv2.imshow('frame', frame)

        key = cv2.waitKey(1)

        # the 't' button is set as the takeoff button
        if key & 0xFF == ord('t') and tookoff == False:
            tello.takeoff()
            print(tello.get_battery())
            tookoff = True

        # the 's' button is set as the "starting to track ball" button
        if key & 0xFF == ord('s') and started == False and tookoff == True:
            started = True

        # the 'q' button is set as the quitting button
        if key & 0xFF == ord('q'):
            break
    
    tello.land()
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


def trackBall_1D(tello, x_drone, y_drone, x_ball, y_ball):
    # This function assumes that the drone cam is pointed at the camera, and that the computer cam is flipped.
    for_back = 0
    up_down = 0
    x_ball_rel = x_ball - x_drone # x and y coordinates that are relative to the ball
    y_ball_rel = y_ball - y_drone
    

    if x_ball_rel < 150 :  # 150 pixels for safety. 
        left_right = 60  # ball is in left side of the drone (computer camera is opposite)

    elif x_ball_rel > 150: # ball is in right side of the drone
        left_right = -60

    else:   # drone is under the ball

        # if y_ball_rel < 200:
        #     hit_ball(x_drone)
        #     return
        
        left_right = 0

    if tello.send_rc_control:
        tello.send_rc_control(left_right, for_back, up_down, 0)


def hit_ball(tello, x_ball_rel, y_ball_rel):
    # x and y coordinates are relative to the ball
     if tello.send_rc_control:
        tello.go_xyz_speed(x_ball_rel/3, 0, y_ball_rel/3, 100)
        time.sleep(1.5)
        tello.go_xyz_speed(0, 0, -y_ball_rel/3, 50)
        time.sleep(1.5)




if __name__ == "__main__":
    """tello = Tello()
    tello.connect()
    tello.takeoff()
    time.sleep(8)
    capture_video(tello)
    tello.land()"""

    tello = Tello()
    play_ball(tello)
