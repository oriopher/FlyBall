from cv2 import threshold
from matplotlib import image
import numpy as np
import cv2


class Frame:
    THRESHOLD_SIZE = 8  # pixels
    H_RANGE = 20
    S_RANGE = 30
    V_RANGE = 170

    SEARCH_RANGE = 50  # pixels

    NO_LOWER_BOUNDS = (0, 0, 0)
    NO_UPPER_BOUNDS = (255, 255, 255)

    def __init__(self):
        self.image = None
        self.x = 0
        self.y = 0
        self.lower = Frame.NO_LOWER_BOUNDS
        self.upper = Frame.NO_UPPER_BOUNDS

    def detect_pixel_coordinates(self, search_range):
        search_range = max(1, search_range)
        x_min, x_max, y_min, y_max = 0, self.image.shape[1], 0, self.image.shape[0]
        if self.x != 0 and self.y != 0 and search_range != 0:
            x_min = max(int(self.x - search_range), x_min)
            x_max = min(int(self.x + search_range) + 1, x_max)
            y_min = max(int(self.y - search_range), y_min)
            y_max = min(int(self.y + search_range) + 1, y_max)

        detection_image = self.image[y_min:y_max, x_min:x_max]
        # convert to hsv
        hsv = cv2.cvtColor(detection_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        # define kernel size
        kernel = np.ones((self.THRESHOLD_SIZE, self.THRESHOLD_SIZE), np.uint8)
        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # target = cv2.bitwise_and(img, img, mask = mask)
        # cv2.imshow('target', target)
        # cv2.waitKey(0)

        balloon_pixels = np.argwhere(mask)
        if len(balloon_pixels) == 0:
            self.x, self.y =  0, 0
            return
        x_coor = np.mean(balloon_pixels[:, 1])
        y_coor = np.mean(balloon_pixels[:, 0])

        self.x = x_coor + x_min
        self.y = y_coor + y_min


    def detect_color(self):
        y_shape = self.image.shape[0]
        x_shape = self.image.shape[1]
        crop_img = self.image[int(y_shape / 3): int(2 * y_shape / 3), int(x_shape / 3): int(2 * x_shape / 3)]
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        ball_color = (int(np.median(h)), int(np.median(s)), int(np.median(v)))
        self.lower = (max(0, ball_color[0] - Frame.H_RANGE), max(0, ball_color[1] - Frame.S_RANGE),
                      max(20, ball_color[2] - Frame.V_RANGE))
        self.upper = (min(255, ball_color[0] + Frame.H_RANGE), min(255, ball_color[1] + Frame.S_RANGE),
                      min(255, ball_color[2] + Frame.V_RANGE))


    @property
    def color_str(self):
        return "%.0f,%.0f,%.0f\n%.0f,%.0f,%.0f\n" % \
               (self.lower[0], self.lower[1], self.lower[2], self.upper[0], self.upper[1], self.upper[2])

    @staticmethod
    def str_to_color_bound(bound):
        bound = bound.split(',')
        return int(bound[0]), int(bound[1]), int(bound[2])

    def save_bounds(self, lower, upper):
        self.lower = self.str_to_color_bound(lower)
        self.upper = self.str_to_color_bound(upper)

    def set_image(self, image):
        self.image = image
        self.THRESHOLD_SIZE = image.shape[1] // 120
        self.SEARCH_RANGE = image.shape[1] // 20
