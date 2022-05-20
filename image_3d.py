from color_bounds import ColorBounds
from frame import Frame
import numpy as np
from camera import Camera
import datetime


class Image3D:

    SEARCH_RANGE_SCALE = 50/300  # when balloon is 300 cm from cameras, search range is 50 px

    def __init__(self, image_left, image_right, phys_x_balloon=0, phys_y_balloon=0, phys_x_drone=0, phys_y_drone=0):
        self.frame_left = Frame(image_left)
        self.frame_right = Frame(image_right)
        self.phys_x_balloon = phys_x_balloon
        self.phys_y_balloon = phys_y_balloon
        self.phys_x_drone = phys_x_drone
        self.phys_y_drone = phys_y_drone
        self.velocity_x_balloon = 0
        self.velocity_y_balloon = 0
        self.velocity_x_drone = 0
        self.velocity_y_drone = 0
        self.time = datetime.datetime.now()


    def calculate_distance(self, left: Camera, right: Camera, d, x_left, x_right, method):
        x, y = 0, 0
        p_left = (self.frame_left.image.shape[1] / 2) / np.tan(left.fov / 2)
        angle_left = np.pi / 2 - np.arctan2(left.flip*(x_left - self.frame_left.image.shape[1] / 2), p_left)
        p_right = (self.frame_right.image.shape[1] / 2) / np.tan(right.fov / 2)
        angle_right = np.pi / 2 - np.arctan2(right.flip*(-x_right + self.frame_right.image.shape[1] / 2), p_right)
        if method == 'parallel':
            # left camera is at (0,0) and right at (0,d)
            x = d * np.tan(angle_right) / (np.tan(angle_right) + np.tan(angle_left))
            y = x * np.tan(angle_left)
        elif method == 'perpendicular':
            # left camera is at (0,0) and right at (d[0],d[1])
            x = (d[0] - d[1] * np.tan(angle_left)) / (1 - np.tan(angle_right) * np.tan(angle_left))
            y = x * np.tan(angle_left)
        # print("a={}, b={}, x={}, y={}, w={}, p={}".format(np.degrees(angle_left), np.degrees(angle_right), self.phys_x, self.phys_y, x_left, x_right))
        return x, y

    
    def calculate_balloon_distance(self, left: Camera, right: Camera, d, method='parallel'):
        x_left, x_right = self.frame_left.x_balloon, self.frame_right.x_balloon
        self.phys_x_balloon, self.phys_y_balloon = self.calculate_distance(left, right, d, x_left, x_right, method)


    def calculate_drone_distance(self, left: Camera, right: Camera, d, method='parallel'):
        x_left, x_right = self.frame_left.x_drone, self.frame_right.x_drone
        self.phys_x_drone, self.phys_y_drone = self.calculate_distance(left, right, d, x_left, x_right, method)

    
    def detect_all(self, colors: ColorBounds, image_old):
        search_range_balloon = Image3D.SEARCH_RANGE_SCALE * image_old.phys_y_balloon
        search_range_drone = Image3D.SEARCH_RANGE_SCALE * image_old.phys_y_drone
        self.frame_left.detect_balloon(colors.ball_left, search_range_balloon, image_old.frame_left.x_balloon, image_old.frame_left.y_balloon)
        self.frame_right.detect_balloon(colors.ball_right, search_range_balloon, image_old.frame_right.x_balloon, image_old.frame_right.y_balloon)
        self.frame_left.detect_drone(colors.drone_left, search_range_drone, image_old.frame_left.x_drone, image_old.frame_left.y_drone)
        self.frame_right.detect_drone(colors.drone_right, search_range_drone, image_old.frame_right.x_drone, image_old.frame_right.y_drone)

    def calculate_all_distances(self, left: Camera, right: Camera, d, method='parallel'):
        balloon_exist, drone_exist = False, False
        if self.frame_left.x_balloon!=0 and self.frame_right.x_balloon!=0:
            balloon_exist = True
            self.calculate_balloon_distance(left, right, d, method)
        if self.frame_left.x_drone!=0 and self.frame_right.x_drone!=0:
            drone_exist = True
            self.calculate_drone_distance(left, right, d, method)

        return balloon_exist, drone_exist

    def calculate_velocities(self, image_old):
        diff_time = self.time - image_old.time
        diff_time_sec = diff_time.microseconds * 1e-6 + diff_time.milliseconds * 1e-3
        self.velocity_x_balloon = (self.phys_x_balloon - image_old.phys_x_balloon) / diff_time_sec
        self.velocity_y_balloon = (self.phys_y_balloon - image_old.phys_y_balloon) / diff_time_sec
        self.velocity_x_drone = (self.phys_x_drone - image_old.phys_x_drone) / diff_time_sec
        self.velocity_y_drone = (self.phys_y_drone - image_old.phys_y_drone) / diff_time_sec
