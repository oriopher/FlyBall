import numpy as np
import cv2


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
NO_UPPER_BOUNDS = (255,255, 255)


def capture_video():
    vid = cv2.VideoCapture(0)
  
    while(True):  
        # Capture the video frame by frame
        ret, frame = vid.read()
    
        # Prosses frame
        x_coor, y_coor = find_balloon_coordinates(frame, PINK_LOWER_BOUNDS, PINK_UPPER_BOUNDS)
        if x_coor!=0 and y_coor!=0:
            frame = cv2.circle(frame, (int(x_coor), int(y_coor)), 15, (0,0,100), 3)

        # Display the resulting frame
        cv2.imshow('frame', frame)

        # the 'q' button is set as the quitting button
        if cv2.waitKey(1) & 0xFF == ord('q'):
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