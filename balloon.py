import numpy as np
import cv2

THRESHOLD_SIZE = 15
BALLOON_H_RANGE = 20
BALLOON_S_RANGE = 20
BALLOON_V_RANGE = 150
# efrat you know hsv change the ranges

RED_LOWER_BOUNDS = (136, 87, 111)
RED_UPPER_BOUNDS = (180, 255, 255)
GREEN_LOWER_BOUNDS = (25, 52, 72)
GREEN_UPPER_BOUNDS = (102, 255, 255)
BLUE_LOWER_BOUNDS = (94, 80, 2)
BLUE_UPPER_BOUNDS = (120, 255, 255)
NO_LOWER_BOUNDS = (0, 0, 0)
NO_UPPER_BOUNDS = (255,255, 255)


def detect_balloon_color(frame):
    y_shape = frame.shape[0]
    x_shape = frame.shape[1]
    crop_img = frame[int(y_shape/4) : int(3*y_shape/4), int(x_shape/4) : int(3*x_shape/4)]
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
    min_color = (max(0, ball_color[0] - BALLOON_H_RANGE), 
                    max(0, ball_color[1] - BALLOON_S_RANGE), 
                    max(20, ball_color[2] - BALLOON_V_RANGE))
    max_color = (min(255, ball_color[0] + BALLOON_H_RANGE), 
                    min(255, ball_color[1] + BALLOON_S_RANGE), 
                    min(255, ball_color[2] + BALLOON_V_RANGE))
    return min_color, max_color


def capture_video():
    vid_web = cv2.VideoCapture(0)
    vid_phone = cv2.VideoCapture(1)

    # the angle of the camera. to change according to used camera.
    p_theta = 33
    w_theta = 33

    distance_web = float(input("Enter distance (in cm) from web cam: "))
    distance_phone = float(input("Enter distance (in cm) from phone cam: "))

    i = 0
    balloon_upper_bound_web = NO_UPPER_BOUNDS
    balloon_lower_bound_web = NO_LOWER_BOUNDS

    balloon_upper_bound_phone = NO_UPPER_BOUNDS
    balloon_lower_bound_phone = NO_LOWER_BOUNDS
  
    while(True):  
        i = i+1
        # Capture the video frame by frame
        ret_web, frame_web = vid_web.read()
        ret_phone, frame_phone = vid_phone.read()

        if i%100 == 0:
            print(balloon_upper_bound_web, balloon_lower_bound_web)
    
        # Process frame
        x_coor_web, y_coor_web = find_balloon_coordinates(frame_web, balloon_lower_bound_web, balloon_upper_bound_web)
        if x_coor_web!=0 and y_coor_web!=0:
            frame_web = cv2.circle(frame_web, (int(x_coor_web), int(y_coor_web)), 15, (0,0,100), 3)
            # updating distance from camera
            if i > 1:
                distance_phone += change_in_distance(distance_web, x_coor_web - x_coor_web_old, frame_web.shape[1], w_theta)
            x_coor_web_old = x_coor_web


        x_coor_phone, y_coor_phone = find_balloon_coordinates(frame_phone, balloon_lower_bound_phone, balloon_upper_bound_phone)
        if x_coor_phone!=0 and y_coor_phone!=0:
            frame_phone = cv2.circle(frame_phone, (int(x_coor_phone), int(y_coor_phone)), 15, (0,0,100), 3)
            if i > 1:
                distance_web += change_in_distance(distance_phone, x_coor_phone - x_coor_phone_old, frame_phone.shape[1], p_theta)
            x_coor_phone_old = x_coor_phone

        # y_shape = frame.shape[0]
        # x_shape = frame.shape[1]
        
        # frame = cv2.rectangle(frame, (int(x_shape/4), int(y_shape/4)), (int(x_shape*3/4), int(y_shape*3/4)), (0,0,100), 3)

        # Display the resulting frame
        cv2.imshow('webcam', frame_web)

        cv2.imshow('phone', frame_phone)

        key = cv2.waitKey(1)
        # the 'w' button is set as the detect color of balloon in the web cam
        if key & 0xFF == ord('w'):
            balloon_lower_bound_web, balloon_upper_bound_web = detect_balloon_color(frame_web)

        # the 'p' button is set as the detect color of balloon in the phone cam
        if key & 0xFF == ord('p'):
            balloon_lower_bound_phone, balloon_upper_bound_phone = detect_balloon_color(frame_phone)

        # the 'q' button is set as the quitting button
        if key & 0xFF == ord('q'):
            break
    
    # After the loop release the cap object
    vid_web.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


def find_balloon_coordinates(img, lower_bound, upper_bound):
    # convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    #define kernel size  
    kernel = np.ones((THRESHOLD_SIZE,THRESHOLD_SIZE), np.uint8)
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


# return updated distance from cam1 using pixels diffrence in cam2 
def change_in_distance(distance, pixels_diff, num_pixels, theta):
    return pixels_diff * pixel_to_cm(distance, num_pixels, theta) # we have to decide about directions beacuse it dependes on sign


# return how much cm in one pixel. (i wrote this function sepperatly beacuse it may be useful later)
def pixel_to_cm(distance, num_pixels, theta):  # notice that theta is actually 0.5 of the total angle
    return distance * 2 * np.tan(theta) / num_pixels


if __name__ == "__main__":
    capture_video()