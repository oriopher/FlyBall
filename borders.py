import os
from camera import Camera
from recognizable_object import RecognizableObject
from quadrangle import QUADRANGLE
import numpy as np


class Borders:
    def __init__(self):
        self.quad = QUADRANGLE()
        self.index = 0              
        self.x_middle = 0                          
        self.y_middle = 0                              
        self.set_borders = False       


    # saves the image in an array
    def set_image(self, balloon: RecognizableObject, left_cam: Camera):
        if self.index <= 3:
            self.quad.coordinates[self.index] = np.array([balloon.x, balloon.y])
            self.index += 1
        if self.index == 4:
            self.quad.fov_horz = left_cam.fov_horz
            self.quad.fov_vert = left_cam.fov_vert
            self.quad.x_n_pix = balloon.frame_left.image.shape[1]
            self.quad.z_n_pix = balloon.frame_left.image.shape[0]
            self.calc_borders()

    # TODO: has a bug!
    # sorts the 4 coordinates before calculating edges
    def sort_coordinates(self):
        tmp_coordinates = np.copy(self.quad.coordinates)
        sorted_coor = np.zeros((4, 2))

        for i in range(len(self.quad.coordinates)):
            idx = np.where(tmp_coordinates[:,1] == np.amin(tmp_coordinates[:,1]))
            sorted_coor[i] = [self.quad.coordinates[idx][0], self.quad.coordinates[idx][1]]
            tmp_coordinates = np.delete(tmp_coordinates, idx)

        if sorted_coor[1][0] <= sorted_coor[0][0]:
            self.quad.coordinates[0][0], self.quad.coordinates[0][1] = sorted_coor[0][0], sorted_coor[0][1]
            self.quad.coordinates[1][0], self.quad.coordinates[1][1] = sorted_coor[1][0], sorted_coor[1][1]         

        else:
            self.quad.coordinates[0][0], self.quad.coordinates[0][1] = sorted_coor[1][0], sorted_coor[1][1]
            self.quad.coordinates[1][0], self.quad.coordinates[1][1] = sorted_coor[0][0], sorted_coor[0][1]            

        if sorted_coor[3][0] <= sorted_coor[2][0]:
            self.quad.coordinates[2][0], self.quad.coordinates[2][1] = sorted_coor[2][0], sorted_coor[2][1]
            self.quad.coordinates[3][0], self.quad.coordinates[3][1] = sorted_coor[3][0], sorted_coor[3][1]         

        else:
            self.quad.coordinates[2][0], self.quad.coordinates[2][1] = sorted_coor[3][0], sorted_coor[3][1]
            self.quad.coordinates[3][0], self.quad.coordinates[3][1] = sorted_coor[2][0], sorted_coor[2][1]     


    def calc_borders(self):
        # self.sort_coordinates()
        self.quad.calc_edges()
        self._calc_middle()


    # calculate the middle coordinates
    def _calc_middle(self):
        self.x_middle = (self.quad.coordinates[0][0] + self.quad.coordinates[1][0]) / 2
        self.y_middle = (self.quad.coordinates[0][1] + self.quad.coordinates[1][1] + self.quad.coordinates[2][1] +
                           self.quad.coordinates[3][1]) / 4    


    # checks if a given coordinate is in borders
    def coordinate_in_borders(self, x, y):
        return self.quad.coordinate_in_quadrangle(x, y)


    # checks if recognizable_object is in borders
    def in_borders(self, recognizable_object: RecognizableObject):
        return self.coordinate_in_borders(recognizable_object.x, recognizable_object.y)


    # draws borders on left frame
    def draw_borders(self, show_img, balloon, color_in=(240, 0, 240), color_out=(240, 0, 240)):
        color = color_in
        if not self.in_borders(balloon):
            color = color_out

        if self.set_borders:
            show_img = self.quad.draw_quadrangle(show_img, color)
        return show_img


    # saves borders coordinates to filename
    def save_borders(self, filename):
        file_text = self._coordinate_str(self.quad.coordinates[0]) + self._coordinate_str(self.quad.coordinates[1]) \
                    + self._coordinate_str(self.quad.coordinates[2]) + self._coordinate_str(self.quad.coordinates[3]) \
                    + self._img_info_str()

        if os.path.exists(filename):
            os.remove(filename)
        with open(filename, 'w') as f:
            f.write(file_text)

        self.set_borders = True
        print("Borders Saved")


    # loads borders from filename
    def load_borders(self, filename):
        if not os.path.exists(filename):
            print("ERROR: borders file does not exist")
            return

        with open(filename, 'r') as f:
            lines = f.readlines()

        self._read_coor(0, lines[0])
        self._read_coor(1, lines[1])
        self._read_coor(2, lines[2])
        self._read_coor(3, lines[3])
        self._read_img_info(lines[4])

        self.calc_borders()
        self.set_borders = True
        print("Borders Loaded")


    def _coordinate_str(self, coordinate):
        return "%.2f,%.2f\n" % (coordinate[0], coordinate[1])


    def _img_info_str(self):
        return "%.3f,%.3f,%.0f,%.0f\n" % (self.quad.fov_horz, self.quad.fov_vert, self.quad.x_n_pix, self.quad.z_n_pix)


    def _read_coor(self, idx, line):
        line_s = line.split(',')
        self.quad.coordinates[idx][0] = float(line_s[0])
        self.quad.coordinates[idx][1] = float(line_s[1])


    def _read_img_info(self, line):
        line_s = line.split(',')
        self.quad.fov_horz = float(line_s[0])
        self.quad.fov_vert = float(line_s[1])
        self.quad.x_n_pix = int(line_s[2])
        self.quad.z_n_pix = int(line_s[3])    