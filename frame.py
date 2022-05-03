import numpy as np
import cv2

THRESHOLD_SIZE = 10
H_RANGE = 30
S_RANGE = 40
V_RANGE = 150

SEARCH_RANGE = 200 # pixels

class Frame:
    def __init__(self, image):
        self.image = image
        self.x_drone = 0
        self.y_drone = 0
        self.x_balloon = 0
        self.y_balloon = 0

    def detect_coordinates(self, bounds, x_old, y_old):
        x_min, x_max, y_min, y_max = 0, self.image.shape[1], 0, self.image.shape[0]
        if x_old!=0 and y_old!=0:
            x_min = np.max(x_old - SEARCH_RANGE, 0)
            x_max = np.min(x_old + SEARCH_RANGE, self.image.shape[1])
            y_min = np.max(y_old - SEARCH_RANGE, 0)
            y_max = np.min(y_old + SEARCH_RANGE, self.image.shape[0])

        detection_image = self.image[y_min:y_max, x_min:x_max]

        # convert to hsv
        hsv = cv2.cvtColor(detection_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, bounds.lower, bounds.upper)

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

        return x_coor + x_min, y_coor + y_min

    def detect_balloon(self, bounds, x_old=0, y_old=0):
        x_coor, y_coor = self.detect_coordinates(bounds, x_old, y_old)
        self.x_balloon = x_coor
        self.y_balloon = y_coor

    def detect_drone(self, bounds, x_old=0, y_old=0):
        x_coor, y_coor = self.detect_coordinates(bounds, x_old, y_old)
        self.x_drone = x_coor
        self.y_drone = y_coor

    def detect_color(self):
        y_shape = self.image.shape[0]
        x_shape = self.image.shape[1]
        crop_img = self.image[int(y_shape/3) : int(2*y_shape/3), int(x_shape/3) : int(2*x_shape/3)]
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        h = hsv[:,:,0]
        s = hsv[:,:,1]
        v = hsv[:,:,2]
        ball_color = (int(np.median(h)), int(np.median(s)), int(np.median(v)))
        min_color = (max(0, ball_color[0] - H_RANGE), max(0, ball_color[1] - S_RANGE), max(20, ball_color[2] - V_RANGE))
        max_color = (min(255, ball_color[0] + H_RANGE), min(255, ball_color[1] + S_RANGE), min(255, ball_color[2] + V_RANGE))
        return [min_color, max_color]

    def show_image(self, window_name, detection_sign = True):
        show_img = self.image
        if detection_sign and self.x_balloon!=0 and self.y_balloon!=0:
            show_img = cv2.circle(show_img, (int(self.x_balloon), int(self.y_balloon)), 15, (0,0,100), 3)
        if detection_sign and self.x_drone!=0 and self.y_drone!=0:
            show_img = cv2.circle(show_img, (int(self.x_drone), int(self.y_drone)), 15, (0,100,0), 3)

        cv2.imshow(window_name, show_img)


class ColorBounds:

    def __init__(self, lower, upper):
        self.lower = lower
        self.upper = upper

    def change(self, lower, upper):
        self.__init__(lower, upper)

