import numpy as np
import cv2

class Frame:
    THRESHOLD_SIZE = 8  # pixels
    H_RANGE = 30
    S_RANGE = 50
    V_RANGE = 170

    SEARCH_RANGE = 50  # pixels

    def __init__(self, image):
        self.image = image
        self.x = 0
        self.y = 0
        self.THRESHOLD_SIZE = self.image.shape[1]/80
        self.SEARCH_RANGE = self.image.shape[1]/20

    def _detect_coordinates(self, bounds, x_old, y_old, search_range):
        search_range = max(1, search_range)
        x_min, x_max, y_min, y_max = 0, self.image.shape[1], 0, self.image.shape[0]
        if x_old != 0 and y_old != 0 and search_range!=0:
            x_min = max(int(x_old - search_range), x_min)
            x_max = min(int(x_old + search_range) + 1, x_max)
            y_min = max(int(y_old - search_range), y_min)
            y_max = min(int(y_old + search_range) + 1, y_max)

        detection_image = self.image[y_min:y_max, x_min:x_max]
        # convert to hsv
        hsv = cv2.cvtColor(detection_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, bounds.lower, bounds.upper)
        # define kernel size
        kernel = np.ones((Frame.THRESHOLD_SIZE, Frame.THRESHOLD_SIZE), np.uint8)
        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # target = cv2.bitwise_and(img, img, mask = mask)
        # cv2.imshow('target', target)
        # cv2.waitKey(0)

        balloon_pixels = np.argwhere(mask)
        if len(balloon_pixels) == 0:
            return 0, 0
        x_coor = np.mean(balloon_pixels[:, 1]) 
        y_coor = np.mean(balloon_pixels[:, 0])

        return x_coor + x_min, y_coor + y_min

    def detect(self, bounds, search_range, x_old=0, y_old=0):
        self.x, self.y = self._detect_coordinates(bounds, x_old, y_old, search_range)

    def detect_color(self):
        y_shape = self.image.shape[0]
        x_shape = self.image.shape[1]
        crop_img = self.image[int(y_shape / 3): int(2 * y_shape / 3), int(x_shape / 3): int(2 * x_shape / 3)]
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        ball_color = (int(np.median(h)), int(np.median(s)), int(np.median(v)))
        min_color = (max(0, ball_color[0] - Frame.H_RANGE), max(0, ball_color[1] - Frame.S_RANGE),
                     max(20, ball_color[2] - Frame.V_RANGE))
        max_color = (min(255, ball_color[0] + Frame.H_RANGE), min(255, ball_color[1] + Frame.S_RANGE),
                     min(255, ball_color[2] + Frame.V_RANGE))
        return [min_color, max_color]



