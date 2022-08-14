from images.camera import Camera
from object_in_frame import ObjectInFrame
import numpy as np
import datetime


class RecognizableObject:
    """
    A class representing a recognizable object in our video capture (a balloon - free or on a drone)
    """
    # Number of previous measured locations of the object used to calculate it's velocity.
    NUM_OF_PREVS = 10

    def __init__(self, text_color, name):
        """
        Initializes the recognizable object.
        :param text_color: the color that will be representing the object in displays.
        :param name: the name of the object.
        """
        self.frame_left = ObjectInFrame()
        self.frame_right = ObjectInFrame()
        self.x = self.y = self.z = 0
        self.vx = self.vy = self.vz = 0
        self.text_colors = text_color
        self.object_exists = False
        self.time = 0
        self.prev_coordinates = np.zeros((0, 3))
        self.prev_times = np.zeros(0, dtype='datetime64')
        self.name = name

    @property
    def colors_string(self):
        """
        A string representations of the color bounds of the object in each cameras frame.
        :return:
        """
        return self.frame_left.color_str + self.frame_right.color_str

    def save_colors_side(self, lower, upper, side):
        """
        Saves the color bounds of the object in the inputted side (the side of the camera).
        :param lower: the lower bound of the colors.
        :param upper: the upper bound of the colors.
        :param side: the ObjectInFrame representing the side in which to save the bounds.
        """
        side.save_bounds(lower, upper)

    def save_colors(self, bounds):
        """
        Saves the color bounds of the object.
        :param bounds: the color bounds to save in the format (lower left, upper left, lower right, upper right).
        """
        sides = [self.frame_left, self.frame_right]
        for i, side in enumerate(sides):
            self.save_colors_side(bounds[i * len(sides)], bounds[i * len(sides) + 1], side)

    def set_frames(self, capture_left, capture_right):
        """
        Sets the frames of last captures by the cameras to the objects ObjectInFrame's.
        :param capture_left: the Frame of the last capture from the left camera.
        :param capture_right: the Frame of the last capture from the right camera.
        """
        self.frame_left.set_frame(capture_left)
        self.frame_right.set_frame(capture_right)

    def _detect_pixel_coordinates(self):
        """
        Detects the objects location in pixel on each side.
        """
        self.frame_left.detect_pixel_coordinates(self.y)
        self.frame_right.detect_pixel_coordinates(self.y)

    def detect_and_set_coordinates(self, left: Camera, right: Camera, d):
        """
        Detects and sets the real life coordinates and velocity of the object.
        :param left: the left camera's Camera object.
        :param right: the right camera's Camera object.
        :param d: the distance between the cameras
        """
        self.time = datetime.datetime.now()
        self.set_frames(left.last_capture, right.last_capture)
        self._detect_pixel_coordinates()
        self.object_exists = False
        if self.frame_left.x != 0 and self.frame_right.x != 0:
            self.object_exists = True
            self._calculate_coordinates(left, right, d)
            # Calculates the object's velocity using the NUM_OF_PREVS last coordinates and the times they where set.
            now = np.array([self.time], dtype='datetime64')
            here = [self.x, self.y, self.z]
            if len(self.prev_times) == RecognizableObject.NUM_OF_PREVS:
                diff_times = self.prev_times - now
                diff_times = diff_times.astype('int') * 10 ** -6
                diff_coordinate = self.prev_coordinates - here
                velocities = (diff_coordinate.T / diff_times).T
                self.vx, self.vy, self.vz = np.mean(velocities, axis=0)
                self.prev_times = np.append(self.prev_times[1:], now)
                self.prev_coordinates = np.vstack([self.prev_coordinates[1:], here])
            else:
                self.prev_times = np.append(self.prev_times, now)
                self.prev_coordinates = np.vstack([self.prev_coordinates, here])

    def _calculate_coordinates(self, left: Camera, right: Camera, d):
        """
        Calculates the real life coordinates of the object
        :param left: the left camera's Camera object.
        :param right: the right camera's Camera object.
        :param d: the distance between the cameras
        """
        x_left, x_right = self.frame_left.x, self.frame_right.x
        self.x, self.y = self._calculate_xy(left, right, d, x_left, x_right)
        z_pix = self.frame_left.y
        # y axis in the frame is actually the z axis in our system
        self.z = self._calculate_z(left, self.y, z_pix)

    def _calculate_xy(self, left: Camera, right: Camera, cam_dist, x_left, x_right):
        """
        Calculates the real life x,y coordinates of the object in centimeters.
        :param left: the left camera's Camera object.
        :param right: the right camera's Camera object.
        :param cam_dist: the distance between the cameras
        :param x_left: the x pixel location of the object in the left frame.
        :param x_right: the x pixel location of the object in the right frame.
        :return: the x,y coordinates of the object.
        """
        # p is just a used for calculation, see documentation.
        p_left = (self.frame_left.image.shape[1] / 2) / np.tan(left.fov_horz / 2)
        angle_left = np.pi / 2 - np.arctan2(left.flip * (x_left - self.frame_left.image.shape[1] / 2), p_left)
        p_right = (self.frame_right.image.shape[1] / 2) / np.tan(right.fov_horz / 2)
        angle_right = np.pi / 2 - np.arctan2(right.flip * (-x_right + self.frame_right.image.shape[1] / 2), p_right)
        # left_cam camera is at (0,0) and right_cam at (0,d)
        x = cam_dist * np.tan(angle_right) / (np.tan(angle_right) + np.tan(angle_left))
        y = x * np.tan(angle_left)
        return x, y

    def _calculate_z(self, cam: Camera, y_cm, z_pix):
        """
        Calculates the real life z point of the object in centimeters.
        :param cam: one of the camera's Camera object (arbitrarily the left camera).
        :param y_cm: the real life y point of the objet
        :param z_pix: the z pixel location of the object in the frame of the inputted camera.
        :return: the z point of the object.
        """
        n_pixels = self.frame_right.image.shape[0]  # Number of pixels in z axis.
        p = (n_pixels / 2) / np.tan(cam.fov_vert / 2)
        return y_cm * (n_pixels / 2 - z_pix) / p

    def detect_color(self, is_left):
        """
        Detects the color of the object in one side.
        :param is_left: whether to detect the objects colors in the left side.
        """
        if is_left:
            self.frame_left.detect_color()
        else:
            self.frame_right.detect_color()
