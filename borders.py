import os
from camera import Camera
from recognizable_object import RecognizableObject
from quadrangle import Quadrangle
import numpy as np


class Borders:
    """
    A class representing the borders of the game (a quadrangle).
    """
    def __init__(self):
        """
        Initializes the borders.
        """
        self.quad = None
        self._temp_coordinates = np.zeros((4, 2), dtype=float)
        self.index = 0
        self.x_middle_1, self.x_middle_2 = 0, 0
        self.y_middle = 0
        self.is_set = False

        # saves the image in an array

    @property
    def coordinates(self):
        """
        :return: the coordinates of the corners of the borders quadrangle,
                 (if all coordinates are set than they are returned int order as depicted in Quadrangle).
        """
        return self.quad.coordinates if self.quad else self._temp_coordinates

    def set_corner(self, balloon: RecognizableObject, left_cam: Camera):
        """
        Sets the coordinates of a corner (as the current location of the balloon).
        if all corners (4) are set - than the borders are calculated.
        :param balloon: the RecognizableObject of the balloon.
        :param left_cam: the Camera of the left camera.
        """
        if self.index <= 3:
            self._temp_coordinates[self.index] = np.array([balloon.x, balloon.y])
            self.index += 1
        if self.index == 4:
            self.full_initialization(left_cam)

    def full_initialization(self, left_cam):
        """
        Performs an initialization of the borders (after all corners are set).
        :param left_cam: the left cameras Camera.
        """
        self.quad = Quadrangle(self._temp_coordinates, left_cam)
        self._set_homes()
        self.is_set = True

    def _set_homes(self):
        """
        Calculates and sets the homes for 2 drones according to the borders.
        """
        self.x_middle_1 = (self.coordinates[0][0] + self.coordinates[1][0]) * (2 / 9)
        self.x_middle_2 = (self.coordinates[0][0] + self.coordinates[1][0]) * (7 / 9)
        self.y_middle = (self.quad.coordinates[0][1] + self.quad.coordinates[1][1] + self.quad.coordinates[2][1] +
                         self.quad.coordinates[3][1]) / 4

    def point_in_borders(self, x, y):
        """
        checks if the inputted point is inside the borders.
        :param x: the x coordinate of the point.
        :param y: the y coordinate of the point.
        :return: True if (x,y) is inside the borders, and False otherwise.
        """
        return self.quad.point_in_quadrangle(x, y)

    def in_borders(self, recognizable_object: RecognizableObject):
        """
        checks if the inputted recognizable object is inside the borders.
        :return: True if (x,y) is inside the borders, and False otherwise.
        :param recognizable_object: the recognizable object in question.
        :return: True if the object is inside the borders, and False otherwise.
        """
        return self.point_in_borders(recognizable_object.x, recognizable_object.y)

    def draw_borders(self, show_img, recognizable_objects, color_in=(240, 0, 240), color_out=(240, 0, 240)):
        """
        Draws the borders on the inputted image (a frame of the left camera).
        :param show_img: the image to draw the borders on.
        :param recognizable_objects: a list of all the recognizable objects in play (balloon and drones).
        :param color_in: the color of the borders if all objects are inside.
        :param color_out: the color of the borders if one of the objects is outside.
        :return: show_img with the borders on it int the correct colors.
        """
        if not self.is_set:
            return show_img
        color = color_in
        for recognizable_object in recognizable_objects:
            if not self.in_borders(recognizable_object):
                color = color_out

        if self.is_set:
            show_img = self.quad.draw_quadrangle(show_img, color)
        return show_img

    def save_borders(self, filename):
        """
        Saves the borders to a file.
        :param filename: the name of the file to save the borders in.
        """
        file_text = str(self.quad)

        if os.path.exists(filename):
            os.remove(filename)
        with open(filename, 'w') as f:
            f.write(file_text)

        print("Borders Saved")

    def load_borders(self, filename, left_cam):
        """
        Loads saved borders from a file and fully initialize the borders.
        :param filename: the name of the file the borders are saved in.
        :param left_cam: the left camera's Camera.
        """
        if not os.path.exists(filename):
            print("ERROR: borders file does not exist")
            return

        with open(filename, 'r') as f:
            lines = f.readlines()

        for i in range(3):
            self._read_coor(i, lines[i])

        self.full_initialization(left_cam)
        print("Borders Loaded")

    def _read_coor(self, idx, line):
        """
        Reads the coordinates of a single saved corner.
        :param idx: the index of the corner.
        :param line: a single line (in the file) of a saved corner.
        """
        line_s = line.split(',')
        for i in range(2):
            self._temp_coordinates[idx][i] = float(line_s[i])
