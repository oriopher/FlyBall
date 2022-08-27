import numpy as np
import cv2


class ObjectInFrame:
    """
    A class representing the location of a specific object in a frame (the frame is updated each cycle).
    """
    H_RANGE = 15
    S_RANGE = 30
    V_RANGE = 170

    NO_LOWER_BOUNDS = (0, 0, 0)
    NO_UPPER_BOUNDS = (255, 255, 255)

    def __init__(self):
        """
        Initialize the object in the frame.
        """
        self.frame = None
        self.x = 0
        self.y = 0
        self.search_range = 0
        self.lower_hsv = ObjectInFrame.NO_LOWER_BOUNDS
        self.upper_hsv = ObjectInFrame.NO_UPPER_BOUNDS

    @property
    def image(self):
        """
        :return: the image of the current frame, the object is in.
        """
        return self.frame.image

    @property
    def color_str(self):
        """
        :return: a string of the color bounds of the object in the frame.
        """
        return "{:.0f},{:.0f},{:.0f}\n{:.0f},{:.0f},{:.0f}\n".format(*self.lower_hsv, *self.upper_hsv)

    def detect_pixel_coordinates(self, distance):
        """
        Detects the pixel coordinates of the object in the frame.
        :param distance: the distance of the object from the camera in the previous frame.
        """
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
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        # define kernel size
        kernel = np.ones((self.frame.threshold_size, self.frame.threshold_size), np.uint8)
        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        balloon_pixels = np.argwhere(mask)
        if len(balloon_pixels) == 0:
            self.x, self.y = 0, 0
            return
        x_coor = np.mean(balloon_pixels[:, 1])
        y_coor = np.mean(balloon_pixels[:, 0])

        self.x = int(x_coor + x_min)
        self.y = int(y_coor + y_min)

    def detect_color(self):
        """
        Detects the color of the object.
        """
        y_shape = self.image.shape[0]
        x_shape = self.image.shape[1]
        crop_img = self.image[int(y_shape / 3): int(2 * y_shape / 3), int(x_shape / 3): int(2 * x_shape / 3)]
        self.set_color_bounds(crop_img)

    def save_bounds(self, lower, upper):
        """
        Saves the objects color bounds of the object to the instance of the class.
        :param lower: the lower bound of the color of the object in the frame.
        :param upper: the upper bound of the color of the object in the frame.
        """
        self.lower_hsv = self.str_to_color_bound(lower)
        self.upper_hsv = self.str_to_color_bound(upper)

    def set_frame(self, frame):
        """
        Sets the frame.
        :param frame: the frame to set.
        """
        self.frame = frame

    @staticmethod
    def str_to_color_bound(bound):
        """
        Transforms a string of color bounds to a tuple of color bound values.
        :param bound:
        :return:
        """
        bound = bound.split(',')
        return int(bound[0]), int(bound[1]), int(bound[2])

    def update_color_bounds(self, pixel_rect=10):
        """
        Updates the color bounds of an object (after first set of bounds).
        :param pixel_rect: half of the side of the square (surrounding the current location)
                           in which the new bounds are detected.
        """
        y_max = min(self.y + pixel_rect, self.image.shape[0])
        y_min = max(self.y - pixel_rect, 0)
        x_max = min(self.x + pixel_rect, self.image.shape[1])
        x_min = max(self.x - pixel_rect, 0)
        crop_img = self.image[y_min: y_max, x_min: x_max]
        self.set_color_bounds(crop_img)

    def set_color_bounds(self, color_image):
        """
        Detects the color of the objects and sets it bounds.
        :param color_image: the image in which the object is the largest object.
        """
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        ball_color = (int(np.median(h)), int(np.median(s)), int(np.median(v)))
        self.lower_hsv = (max(0, ball_color[0] - ObjectInFrame.H_RANGE),
                          max(0, ball_color[1] - ObjectInFrame.S_RANGE),
                          max(20, ball_color[2] - ObjectInFrame.V_RANGE))
        self.upper_hsv = (min(255, ball_color[0] + ObjectInFrame.H_RANGE),
                          min(255, ball_color[1] + ObjectInFrame.S_RANGE),
                          min(255, ball_color[2] + ObjectInFrame.V_RANGE))
