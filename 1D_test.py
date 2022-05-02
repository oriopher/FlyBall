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

DISTANCE = 200

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
    vid = cv2.VideoCapture(0)
    tello.connect()
    tookoff = False
    started = False
    # memory = 0 
    continue_test = True

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
           # memory = 0
        #elif started:
          #  memory = memory + 1

        if tookoff and started:
            # if memory > MEMORY and started:
                #  break
            # move the drone towards the ball
            track_ball_1d(tello, x_drone, y_drone, x_ball, y_ball, DISTANCE, frame.shape[1])

        # Display the resulting frame
        cv2.imshow('frame', frame)

        key = cv2.waitKey(1)

        # the 't' button is set as the takeoff button
        if key & 0xFF == ord('t') and tookoff == False:
            tello.takeoff()
            print("battery = ", tello.get_battery(), "%")
            tookoff = True

        # the 's' button is set as the "starting to track ball" button
        if key & 0xFF == ord('s') and started == False and tookoff == True:
            started = True

        if key & 0xFF == ord('b'):
            balloon_lower_bound, balloon_upper_bound = detect_balloon_color(frame)

        if key & 0xFF == ord('d'):
            drone_lower_bound, drone_upper_bound = detect_balloon_color(frame)

        # the 'q' button is set as the quitting button
        if key & 0xFF == ord('q'):
            continue_test = False
            break

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


def pixel_to_cm(pix_num, distance, num_pixels):
    return pix_num


def track_ball_1d(tello, x_drone, y_drone, x_ball, y_ball, distance, num_pixels):
    # This function assumes that the drone cam is pointed at the camera, and that the computer cam is flipped.
    for_back = 0
    up_down = 0
    x_pix_rel = x_ball - x_drone # x and y coordinates that are relative to the ball
    y_pix_rel = y_ball - y_drone
    a = 1/800
    b = 19/40

    x_cm_rel = pixel_to_cm(x_pix_rel, distance, num_pixels)
    y_cm_rel = pixel_to_cm(y_pix_rel, distance, num_pixels)

    velocity = int(min(a*x_cm_rel**2 + b*x_cm_rel, 60))

    if x_cm_rel < 0:  # 150 pixels for safety. 
        left_right = -1*velocity  # ball is in left side of the drone
        
    elif x_pix_rel > 0: # ball is in right side of the drone
        left_right = velocity

    else:   # drone is under the ball
        left_right = 0
        # if y_ball_rel < 200:
        #     hit_ball(x_drone)
        #     return
    if tello.send_rc_control:
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
