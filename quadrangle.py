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

        # calculate the borders pixels location on frame
        self.pixels_coordinates[3][0], self.pixels_coordinates[3][1] = phys_to_left_pix(self.coordinates[3][0],
                                                                                        self.coordinates[3][1],
                                                                                        FLOOR_HEIGHT - 10, self.x_n_pix,
                                                                                        self.z_n_pix, self.fov_horz,
                                                                                        self.fov_vert)
        self.pixels_coordinates[2][0], self.pixels_coordinates[2][1] = phys_to_left_pix(self.coordinates[2][0],
                                                                                        self.coordinates[2][1],
                                                                                        FLOOR_HEIGHT - 10, self.x_n_pix,
                                                                                        self.z_n_pix, self.fov_horz,
                                                                                        self.fov_vert)
        self.pixels_coordinates[1][0], self.pixels_coordinates[1][1] = phys_to_left_pix(self.coordinates[1][0],
                                                                                        self.coordinates[1][1],
                                                                                        FLOOR_HEIGHT - 10, self.x_n_pix,
                                                                                        self.z_n_pix, self.fov_horz,
                                                                                        self.fov_vert)
        self.pixels_coordinates[0][0], self.pixels_coordinates[0][1] = phys_to_left_pix(self.coordinates[0][0],
                                                                                        self.coordinates[0][1],
                                                                                        FLOOR_HEIGHT - 10, self.x_n_pix,
                                                                                        self.z_n_pix, self.fov_horz,
                                                                                        self.fov_vert)

    
    def cross_quadrangle(point_a, point_b):
        # Checks if the line from a to b crosses the quadrangle.
        return True


