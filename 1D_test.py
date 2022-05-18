import math
from turtle import left
import numpy as np
import cv2
from djitellopy import Tello
from time import sleep

THRESHOLD_SIZE = 15
BALLOON_H_RANGE = 20
BALLOON_S_RANGE = 20
BALLOON_V_RANGE = 150
MEMORY = 30
THRESHOLD_SIZE = 7

DISTANCE = 170
FOV_X = 67
FOV_Y = 55

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

balloon_upper_bound = NO_UPPER_BOUNDS # efrat moved it here so it wont reset with every iteration.
balloon_lower_bound = NO_LOWER_BOUNDS
drone_upper_bound = NO_UPPER_BOUNDS
drone_lower_bound = NO_LOWER_BOUNDS

def detect_balloon_color(frame):
    y_shape = frame.shape[0]
    x_shape = frame.shape[1]
    crop_img = frame[int(y_shape/4) : int(3*y_shape/4) , int(x_shape/4) : int(3*x_shape/4)]
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    h = hsv[:,:,0]
    s = hsv[:,:,1]
    v = hsv[:,:,2]
    ball_color = (int(np.median(h)), int(np.median(s)), int(np.median(v)))
    # min_color = (max(0,int(np.min(h))-BALLOON_H_RANGE), 
    #                 max(0,int(np.min(s))-BALLOON_S_RANGE), 
    #                 max(0,int(np.min(v))-BALLOON_V_RANGE))
    # max_color = (min(255,int(np.max(h))+BALLOON_H_RANGE), 
    #                 min(255,int(np.max(s))+BALLOON_S_RANGE), 
    #                 min(255,int(np.max(v))+BALLOON_V_RANGE))
    min_color = (max(0,ball_color[0]-BALLOON_H_RANGE), 
                    max(0,ball_color[1]-BALLOON_S_RANGE), 
                    max(0,ball_color[2]-BALLOON_V_RANGE))
    max_color = (min(255,ball_color[0]+BALLOON_H_RANGE), 
                    min(255,ball_color[1]+BALLOON_S_RANGE), 
                    min(255,ball_color[2]+BALLOON_V_RANGE))
    return min_color,max_color



def detect_balloon_color(frame):
    y_shape = frame.shape[0]
    x_shape = frame.shape[1]
    crop_img = frame[int(y_shape/4) : int(3*y_shape/4) , int(x_shape/4) : int(3*x_shape/4)]
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    h = hsv[:,:,0]
    s = hsv[:,:,1]
    v = hsv[:,:,2]
    ball_color = (int(np.median(h)), int(np.median(s)), int(np.median(v)))
    min_color = (max(0,ball_color[0]-BALLOON_H_RANGE), 
                    max(0,ball_color[1]-BALLOON_S_RANGE), 
                    max(20,ball_color[2]-BALLOON_V_RANGE))
    max_color = (min(255,ball_color[0]+BALLOON_H_RANGE), 
                    min(255,ball_color[1]+BALLOON_S_RANGE), 
                    min(255,ball_color[2]+BALLOON_V_RANGE))
    return min_color,max_color



def play_ball(tello, bounds):
    vid = cv2.VideoCapture(2)
    tello.connect()
    print(tello.get_battery())
    tookoff = False
    started = False
    # memory = 0 
    continue_test = True
    missed_detection_counter = 0
    i = 0

    balloon_lower_bound = bounds[0]
    balloon_upper_bound = bounds[1]
    drone_lower_bound = bounds[2]
    drone_upper_bound = bounds[3]

    while True:
        # Capture the video frame by frame
        ret, frame = vid.read()
        frame = cv2.flip(frame, 1)

        # recognize ball
        x_ball, y_ball = find_object_coordinates(frame, balloon_lower_bound, balloon_upper_bound)
        if x_ball != 0 and y_ball != 0:
            cv2.circle(frame, (int(x_ball), int(y_ball)), 10, (0, 100, 0), 3)

        # recognize drone
        x_drone, y_drone = find_object_coordinates(frame, drone_lower_bound, drone_upper_bound)

        if x_drone != 0 and y_drone != 0:
            cv2.circle(frame, (int(x_drone), int(y_drone)), 10, (0, 0, 100), 2)

        # For safety if we do not see the drone for 20 frames the frogram will stop
        # if tookoff and (x_drone <= 10 or x_drone >= frame.shape[1] - 10):
        #     missed_detection_counter += 1
        # if missed_detection_counter >= 20 and tookoff:
        #     break


        if tookoff and started:
            # if memory > MEMORY and started:
                #  break
            # move the drone towards the ball
            track_ball_1d(tello, x_drone, y_drone, x_ball, y_ball, DISTANCE, frame.shape[1], FOV_X, FOV_Y)

        # Display the resulting frame
        cv2.imshow('frame', frame)

        key = cv2.waitKey(1)

        # the 't' button is set as the takeoff button
        if key & 0xFF == ord('t') and not tookoff:
            tello.takeoff()
            print("battery = ", tello.get_battery(), "%")
            tookoff = True

        # the 's' button is set as the "starting to track ball" button
        if key & 0xFF == ord('s') and not started and tookoff:
            started = True

        if key & 0xFF == ord('b'):
            balloon_lower_bound, balloon_upper_bound = detect_balloon_color(frame)

        if key & 0xFF == ord('d'):
            drone_lower_bound, drone_upper_bound = detect_balloon_color(frame)

        # the 'q' button is set as the quitting button
        if key & 0xFF == ord('q'):
            continue_test = False
            break
        # 'l' is exit normally
        if key & 0xFF == ord('l'):
            break
    
    tello.land()
    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

    return [balloon_lower_bound, balloon_upper_bound, drone_lower_bound, drone_upper_bound], continue_test
    


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


def pixel_to_cm(pix_coor, distance, num_pixels, fov):
    p = num_pixels  / 2 * math.tan(fov / 2)
    return -distance / p * (pix_coor - num_pixels / 2)


# def get_speed_from_cam(coor_last_frame, coor_this_frame):


def quadratic_velocity(x_cm_rel):
    for_back = 0
    up_down = 0
    a = 5
    b = 19/40

    velocity = int(min(a*x_cm_rel**2 + b*x_cm_rel, 60))

    if x_cm_rel < 20:  # 150 pixels for safety. 
        left_right = -velocity  # ball is in left side of the drone
        
    elif x_cm_rel > 20: # ball is in right side of the drone
        left_right = velocity

    else:   # drone is under the ball
        left_right = 0
        # if y_ball_rel < 200:
        #     hit_ball(x_drone)
        #     return

    return left_right, for_back, up_down
    


def lin_velocity(x_cm_rel):
    for_back = 0
    up_down = 0
    a = 1
    b = 19/40

    velocity = int(min(abs(a*x_cm_rel), 60))

    if x_cm_rel > 5:  # 150 pixels for safety. 
        left_right = -velocity  # ball is in left side of the drone
        
    elif x_cm_rel < -5: # ball is in right side of the drone
        left_right = +velocity

    else:   
        left_right = 0

    return left_right, for_back, up_down


def lin_velocity_with_acc(x_cm_rel):
    #this function assumes the drone is looking at the same direction as the camera.
    for_back = 0
    up_down = 0
    a = 1.5
    b = 1.5
    c = 0.5

    velocity = int(min(abs(a*x_cm_rel), 80))
    real_vel = tello.get_speed_x() * 8  # after multiplying the speed is in cm/s
    print("real_vel: ", real_vel)

    # ball is in left side of the drone and it's not too fast
    if x_cm_rel > 5 and x_cm_rel > -b * real_vel:   # If the velocity is positive we would like to stop
        left_right = -velocity

    # ball is in right side of the drone and it's not too fast
    elif x_cm_rel < -5 and x_cm_rel < -b * real_vel:    # If the velocity is negative we would like to stop
        left_right = velocity

    else:
        if abs(real_vel) >= 10:
            left_right = -real_vel / abs(real_vel) * 50
        else:
            left_right = 0

    return left_right, for_back, up_down



def track_ball_1d(tello, x_drone, y_drone, x_ball, y_ball, distance, num_pixels, fov_x, fov_y):
    # This function assumes that the drone cam is pointed at the camera, and that the computer cam is flipped.
    x_ball_cm = pixel_to_cm(x_ball, distance, num_pixels, fov_x) 
    x_drone_cm = pixel_to_cm(x_drone, distance, num_pixels, fov_x) 

    
    x_cm_rel = x_ball_cm - x_drone_cm
    print("x_ball_cm: ", x_ball_cm, "x_drone_cm: ", x_drone_cm)
    print("x_cm_rel: ", x_cm_rel)
    # y_cm_rel = pixel_to_cm(y_pix_rel, distance, num_pixels)

    left_right, for_back, up_down = 0,0,0
    if tello.send_rc_control:
        # if 20 <= x_cm_rel <= 30:
        #     tello.send_rc_control(left_right, for_back, up_down, 0)
        #     tello.move_right(int(x_cm_rel))
        # elif -30 <= x_cm_rel <= -20:
        #     tello.send_rc_control(left_right, for_back, up_down, 0)
        #     tello.move_left(int(-x_cm_rel))
        # else:
        left_right, for_back, up_down = lin_velocity_with_acc(x_cm_rel)
        tello.send_rc_control(left_right, for_back, up_down, 0)


def hit_ball(tello, x_ball_rel, y_ball_rel):
    # x and y coordinates are relative to the ball
     if tello.send_rc_control:
        tello.go_xyz_speed(x_ball_rel/3, 0, y_ball_rel/3, 100)
        sleep(1.5)
        tello.go_xyz_speed(0, 0, -y_ball_rel/3, 50)
        sleep(1.5)
        

if __name__ == "__main__":
    tello = Tello()
    balloon_upper_bound = NO_UPPER_BOUNDS
    balloon_lower_bound = NO_LOWER_BOUNDS
    drone_upper_bound = NO_UPPER_BOUNDS
    drone_lower_bound = NO_LOWER_BOUNDS
    bounds = [balloon_lower_bound, balloon_upper_bound, drone_lower_bound, drone_upper_bound]

    while True:
        bounds, continue_test = play_ball(tello, bounds)
        if not continue_test:
            break
