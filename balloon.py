import numpy as np
import cv2

THRESHOLD_SIZE = 15
BALLOON_H_RANGE = 20
BALLOON_S_RANGE = 20
BALLOON_V_RANGE = 20
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


def capture_video():
    vid = cv2.VideoCapture(0)
    i = 0
    balloon_upper_bound = NO_UPPER_BOUNDS
    balloon_lower_bound = NO_LOWER_BOUNDS
  
    while(True):  
        i = i+1
        # Capture the video frame by frame
        ret, frame = vid.read()

        if i%100 == 0:
            print(1,balloon_lower_bound, balloon_upper_bound)
    
        # Prosses frame
        x_coor, y_coor = find_balloon_coordinates(frame, balloon_lower_bound, balloon_upper_bound)
        if x_coor!=0 and y_coor!=0:
            frame = cv2.circle(frame, (int(x_coor), int(y_coor)), 15, (0,0,100), 3)

        # Display the resulting frame
        cv2.imshow('frame', frame)

        key = cv2.waitKey(1)
        # the 'b' button is set as the detect color of balloon
        if key & 0xFF == ord('b'):
            balloon_lower_bound, balloon_upper_bound = detect_balloon_color(frame)
            print(2,balloon_lower_bound, balloon_upper_bound)

        # the 'q' button is set as the quitting button
        if key & 0xFF == ord('q'):
            break
    
    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


def find_balloon_coordinates(img, lower_bound, upper_bound):
    # convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    #define kernel size  
    kernel = np.ones((THRESHOLD_SIZE,THRESHOLD_SIZE),np.uint8)
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


if __name__ == "__main__":
    capture_video()