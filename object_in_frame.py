import numpy as np
import cv2


class ObjectInFrame:
    H_RANGE = 20
    S_RANGE = 30
    V_RANGE = 170

    NO_LOWER_BOUNDS = (0, 0, 0)
    NO_UPPER_BOUNDS = (255, 255, 255)

    def __init__(self):
        self.frame = None
        self.x = 0
        self.y = 0
        self.threshold_size = 0
        self.search_range = 0
        self.lower = ObjectInFrame.NO_LOWER_BOUNDS
        self.upper = ObjectInFrame.NO_UPPER_BOUNDS

    @property
    def image(self):
        return self.frame.image

    @property
    def color_str(self):
        return "%.0f,%.0f,%.0f\n%.0f,%.0f,%.0f\n" % \
               (self.lower[0], self.lower[1], self.lower[2], self.upper[0], self.upper[1], self.upper[2])

    def detect_pixel_coordinates(self, distance):
        search_range = max(1, self.frame.search_range_scale(distance))
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
        kernel = np.ones((self.threshold_size, self.threshold_size), np.uint8)
        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        balloon_pixels = np.argwhere(mask)
        if len(balloon_pixels) == 0:
            self.x, self.y = 0, 0
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
        self.lower = (max(0, ball_color[0] - ObjectInFrame.H_RANGE), max(0, ball_color[1] - ObjectInFrame.S_RANGE),
                      max(20, ball_color[2] - ObjectInFrame.V_RANGE))
        self.upper = (min(255, ball_color[0] + ObjectInFrame.H_RANGE), min(255, ball_color[1] + ObjectInFrame.S_RANGE),
                      min(255, ball_color[2] + ObjectInFrame.V_RANGE))

    def save_bounds(self, lower, upper):
        self.lower = self.str_to_color_bound(lower)
        self.upper = self.str_to_color_bound(upper)

    def set_image(self, frame):
        self.frame = frame

    @staticmethod
    def str_to_color_bound(bound):
        bound = bound.split(',')
        return int(bound[0]), int(bound[1]), int(bound[2])
