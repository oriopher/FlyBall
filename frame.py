import numpy as np
import cv2
import time

class Frame:
    THRESHOLD_SIZE = 8  # pixels
    H_RANGE = 20
    S_RANGE = 30
    V_RANGE = 150

    SEARCH_RANGE = 70  # pixels

    def __init__(self, image):
        self.image = image
        self.x_drone = 0
        self.y_drone = 0
        self.x_balloon = 0
        self.y_balloon = 0

    def detect_coordinates(self, bounds, x_old, y_old):
        x_min, x_max, y_min, y_max = 0, self.image.shape[1], 0, self.image.shape[0]
        if x_old != 0 and y_old != 0:
            x_min = max(int(x_old - Frame.SEARCH_RANGE), x_min)
            x_max = min(int(x_old + Frame.SEARCH_RANGE) + 1, x_max)
            y_min = max(int(y_old - Frame.SEARCH_RANGE), y_min)
            y_max = min(int(y_old + Frame.SEARCH_RANGE) + 1, y_max)

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
        x_coor = np.median(balloon_pixels[:, 1])
        y_coor = np.median(balloon_pixels[:, 0])

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

    def show_image(self, window_name, detection_sign=True, text_balloon=None, text_drone=None):
        show_img = self.image
        if detection_sign and self.x_balloon != 0 and self.y_balloon != 0:
            show_img = cv2.circle(show_img, (int(self.x_balloon), int(self.y_balloon)), 15, (0, 0, 0), 3)
            if text_balloon:
                show_img = cv2.putText(show_img, text_balloon, (int(self.x_balloon), int(self.y_balloon)),
                                       cv2.FONT_HERSHEY_DUPLEX, 2, (250, 250, 250), 2, cv2.LINE_AA)
        if detection_sign and self.x_drone != 0 and self.y_drone != 0:
            if text_drone:
                show_img = cv2.putText(show_img, text_drone, (int(self.x_drone), int(self.y_drone)),
                                       cv2.FONT_HERSHEY_DUPLEX, 2, (250, 250, 250), 2, cv2.LINE_AA)
            show_img = cv2.circle(show_img, (int(self.x_drone), int(self.y_drone)), 15, (0, 0, 0), 3)

        cv2.imshow(window_name, show_img)


class ColorBound:
    NO_LOWER_BOUNDS = (0, 0, 0)
    NO_UPPER_BOUNDS = (255, 255, 255)

    def __init__(self):
        self.lower = ColorBound.NO_LOWER_BOUNDS
        self.upper = ColorBound.NO_UPPER_BOUNDS

    def change(self, lower, upper):
        self.lower = lower
        self.upper = upper


class ColorBounds:

    def __init__(self):
        self.ball_left = ColorBound()
        self.ball_right = ColorBound()
        self.drone_left = ColorBound()
        self.drone_right = ColorBound()

    def change_ball_left(self, lower, upper):
        self.ball_left.change(lower, upper)

    def change_ball_right(self, lower, upper):
        self.ball_right.change(lower, upper)

    def change_drone_left(self, lower, upper):
        self.drone_left.change(lower, upper)

    def change_drone_right(self, lower, upper):
        self.drone_right.change(lower, upper)


class Image3D:

    def __init__(self, image_left, image_right, phys_x=0, phys_y=0):
        self.frame_left = Frame(image_left)
        self.frame_right = Frame(image_right)
        self.phys_x = phys_x
        self.phys_y = phys_y

    def calculate_distance(self, left, right, d):
        p_left = (self.frame_left.image.shape[1] / 2) / np.tan(left.fov / 2)  # left camera
        angle_left = np.pi / 2 - np.arctan2(left.flip*(self.frame_left.x_balloon - self.frame_left.image.shape[1] / 2), p_left)
        p_right = (self.frame_right.image.shape[1] / 2) / np.tan(right.fov / 2)  # right camera
        angle_right = np.pi / 2 - np.arctan2(right.flip*(-self.frame_right.x_balloon + self.frame_right.image.shape[1] / 2), p_right)
        # left camera is at (0,0)
        self.phys_x = d * np.tan(angle_right) / (np.tan(angle_right) + np.tan(angle_left))
        self.phys_y = self.phys_x * np.tan(angle_left)
        # print("a={}, b={}, x={}, y={}, w={}, p={}".format(np.degrees(angle_left), np.degrees(angle_right), self.phys_x, self.phys_y, self.frame_left.x_balloon, self.frame_right.x_balloon))
