from camera import Camera
from frame import Frame
import numpy as np
import datetime


class RecognizableObject:
    SEARCH_RANGE_SCALE_A = -0.25
    SEARCH_RANGE_SCALE_B = 140
    NUM_OF_PREVS = 10

    def __init__(self, text_colors, radius, name):
        self.frame_left = Frame()
        self.frame_right = Frame()
        self.x = self.y = self.z = 0
        self.vx = self.vy = self.vz = 0
        self.text_colors = text_colors
        self.object_exists = False
        self.time = 0
        self.prev_coordinates = np.zeros((0, 3))
        self.prev_times = np.zeros(0, dtype='datetime64')
        self.radius = radius
        self.name = name

    @property
    def colors_string(self):
        return self.frame_left.color_str + self.frame_right.color_str

    def save_colors_side(self, lower, upper, side):
        side.save_bounds(lower, upper)

    def save_colors(self, bounds):
        sides = [self.frame_left, self.frame_right]
        for i, side in enumerate(sides):
            self.save_colors_side(bounds[i * len(sides)], bounds[i * len(sides) + 1], side)

    def set_frames(self, image_left, image_right):
        self.frame_left.set_image(image_left)
        self.frame_right.set_image(image_right)

    @staticmethod
    def _search_range_scale(distance):
        if distance < 50 or distance > 500:
            return 0
        search = RecognizableObject.SEARCH_RANGE_SCALE_A * distance + RecognizableObject.SEARCH_RANGE_SCALE_B
        return min(max(search, 40), 200)

    def detect_pixel_coordinates(self):
        search_range = self._search_range_scale(self.y)
        self.frame_left.detect_pixel_coordinates(search_range)
        self.frame_right.detect_pixel_coordinates(search_range)

    def detect_and_set_coordinates(self, left: Camera, right: Camera, d):
        self.time = datetime.datetime.now()
        self.set_frames(left.last_capture, right.last_capture)
        self.detect_pixel_coordinates()
        self.object_exists = False
        if self.frame_left.x != 0 and self.frame_right.x != 0:
            self.object_exists = True
            self._calculate_coordinates(left, right, d)
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
        x_left, x_right = self.frame_left.x, self.frame_right.x
        self.x, self.y = self._calculate_xy(left, right, d, x_left, x_right)
        z_pix = self.frame_left.y  # y axis in the frame is actually the z axis in our system
        self.z = self._calculate_z(left, self.y, z_pix)

    def _calculate_xy(self, left: Camera, right: Camera, cam_dist, x_left, x_right):
        # Will return x,y which are the objects coordinates in cm. cam dist is the distance between the cameras.
        # p is just a used for calculation, see documentation.
        x, y = 0, 0
        p_left = (self.frame_left.image.shape[1] / 2) / np.tan(left.fov_horz / 2)
        angle_left = np.pi / 2 - np.arctan2(left.flip * (x_left - self.frame_left.image.shape[1] / 2), p_left)
        p_right = (self.frame_right.image.shape[1] / 2) / np.tan(right.fov_horz / 2)
        angle_right = np.pi / 2 - np.arctan2(right.flip * (-x_right + self.frame_right.image.shape[1] / 2), p_right)
        # left_cam camera is at (0,0) and right_cam at (0,d)
        x = cam_dist * np.tan(angle_right) / (np.tan(angle_right) + np.tan(angle_left))
        y = x * np.tan(angle_left)
        # print("a={}, b={}, x={}, y={}, w={}, p={}".format(np.degrees(angle_left), np.degrees(angle_right), self.phys_x, self.phys_y, x_left, x_right))
        return x, y + self.radius

    def _calculate_z(self, cam: Camera, y_cm, z_pix):
        # Will return the height in cm. Requires y in cm.
        n_pixels = self.frame_right.image.shape[0]  # Number of pixels in z axis.
        p = (n_pixels / 2) / np.tan(cam.fov_vert / 2)
        return y_cm * (n_pixels / 2 - z_pix) / p
