import numpy as np
import cv2
from common import calc_linear_eq, phys_to_left_pix, FLOOR_HEIGHT


class QUADRANGLE:
    def __init__(self):     
        self.m_left = 0                            
        self.b_left = 0                              
        self.m_right = 0                              
        self.b_right = 0                              
        self.m_upper = 0                   
        self.b_upper = 0
        self.m_low = 0
        self.b_low = 0
        self.coordinates = np.zeros((4, 2))
        self.pixels_coordinates = np.zeros((4, 2), dtype=int)
        self.fov_horz = 0
        self.fov_vert = 0
        self.x_n_pix = 0
        self.z_n_pix = 0

    
    def coordinate_in_quadrangle(self, x, y):
        # coordinate is outside upper edge
        if (y - self.m_upper * x - self.b_upper > 0):
            return False

        # coordinate is outside low edge
        if y - self.m_low * x - self.b_low < 0:
            return False

        # coordinate is outside left edge
        if self.m_left >= 0:
            if y - self.m_left * x - self.b_left > 0:
                return False

        elif y - self.m_left * x - self.b_left < 0:
            return False

        # coordinate is outside right edge
        if self.m_right >= 0:
            if y - self.m_right * x - self.b_right < 0:
                return False

        elif y - self.m_right * x - self.b_right > 0:
            return False

        # coordinate is in quadrangle
        return True        


    def calc_edges(self):
        self.m_left, self.b_left = calc_linear_eq(self.coordinates[3], self.coordinates[1])
        self.m_right, self.b_right = calc_linear_eq(self.coordinates[2], self.coordinates[0])
        self.m_upper, self.b_upper = calc_linear_eq(self.coordinates[3], self.coordinates[2])
        self.m_low, self.b_low = calc_linear_eq(self.coordinates[1], self.coordinates[0])

        # calculate the coordinates pixels location on frame
        for i in range(len(self.pixels_coordinates)):
            if self.coordinates[i][1] != 0: 
                self.pixels_coordinates[i][0], self.pixels_coordinates[i][1] = phys_to_left_pix(self.coordinates[i][0],
                                                                                            self.coordinates[i][1],
                                                                                            FLOOR_HEIGHT - 10, self.x_n_pix,
                                                                                            self.z_n_pix, self.fov_horz,
                                                                                            self.fov_vert)


    def sort_coordinates():
        return

    # draws quadrangle on frame
    def draw_quadrangle(self, show_img, color=(240, 0, 240)):
        corners = [0, 1, 3, 2]
        for i, cor in enumerate(corners):
            cor_next = corners[(i + 1) % len(corners)]
            show_img = cv2.line(show_img, (self.pixels_coordinates[cor][0], self.pixels_coordinates[cor][1]),
                            (self.pixels_coordinates[cor_next][0], self.pixels_coordinates[cor_next][1]), color, thickness=2)            

        return show_img