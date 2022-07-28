import os
from image_3d import Image3D
from camera import Camera
from utils import phys_to_left_pix, FLOOR_HEIGHT
import cv2
import numpy as np

class Borders:
    def __init__(self):
        self.index = 0               # (x_4, y_4) ################################ (x_3, y_3)
        self.m_left = 0                             ############################
        self.b_left = 0                               #######################
        self.m_right = 0                                ##################
        self.b_right = 0                                  ##############
        self.m_upper = 0                       # (x_2, y_2) ########## (x_1, y_1)
        self.b_upper = 0
        self.m_low = 0 
        self.b_low = 0
        self.x_middle = 0
        self.y_middle = 0
        self.coordinates = np.zeros((4,2))
        self.pixels_coordinates = np.zeros((4,2), dtype=int)
        self.fov_horz = 0
        self.fov_vert = 0
        self.x_n_pix = 0
        self.z_n_pix = 0
        self.set_borders = False


    # saving the image in the array
    def set_image(self, image_3d: Image3D, left_cam: Camera):
        if (self.index <= 3):
            self.coordinates[self.index] = np.array([image_3d.phys_x_balloon, image_3d.phys_y_balloon])
            self.index += 1
        if self.index == 4:
            self.fov_horz = left_cam.fov_horz
            self.fov_vert = left_cam.fov_vert
            self.x_n_pix = image_3d.frame_left.image.shape[1]
            self.z_n_pix = image_3d.frame_left.image.shape[0]
            self.calc_borders()


    def calc_borders(self):
        self.m_left, self.b_left = self.calc_linear_eq(self.coordinates[3], self.coordinates[1])
        self.m_right, self.b_right = self.calc_linear_eq(self.coordinates[2], self.coordinates[0])
        self.m_upper, self.b_upper = self.calc_linear_eq(self.coordinates[3], self.coordinates[2])
        self.m_low, self.b_low = self.calc_linear_eq(self.coordinates[1], self.coordinates[0])                    

        # calculate the middle coordinates
        self.x_middle = (self.coordinates[0][0] + self.coordinates[1][0]) / 2
        self.y_middle = (self.coordinates[0][1] + self.coordinates[1][1] + self.coordinates[2][1] + self.coordinates[3][1]) / 4

        self.pixels_coordinates[3][0], self.pixels_coordinates[3][1] = phys_to_left_pix(self.coordinates[3][0], self.coordinates[3][1], FLOOR_HEIGHT - 10, self.x_n_pix, self.z_n_pix, self.fov_horz, self.fov_vert)        
        self.pixels_coordinates[2][0], self.pixels_coordinates[2][1] = phys_to_left_pix(self.coordinates[2][0], self.coordinates[2][1], FLOOR_HEIGHT - 10, self.x_n_pix, self.z_n_pix, self.fov_horz, self.fov_vert)        
        self.pixels_coordinates[1][0], self.pixels_coordinates[1][1] = phys_to_left_pix(self.coordinates[1][0], self.coordinates[1][1], FLOOR_HEIGHT - 10, self.x_n_pix, self.z_n_pix, self.fov_horz, self.fov_vert)        
        self.pixels_coordinates[0][0], self.pixels_coordinates[0][1] = phys_to_left_pix(self.coordinates[0][0], self.coordinates[0][1], FLOOR_HEIGHT - 10, self.x_n_pix, self.z_n_pix, self.fov_horz, self.fov_vert)    

    def calc_linear_eq(self, coor1, coor2):
        m = (coor2[1] - coor1[1]) / (coor2[0] - coor1[0])
        b = coor2[1] - m * coor2[0]  
        return m, b


    def coordinat_in_borders(self, x, y):
        # coordinate is too far from camera
        if (y - self.m_upper * x - self.b_upper > 0):
            return False

        # coordinate is too close to camera
        if y - self.m_low * x - self.b_low < 0:
            return False

        # coordinate is out of left border
        if self.m_left >= 0:
            if y - self.m_left * x - self.b_left > 0:
                return False

        elif y - self.m_left * x - self.b_left < 0:   
            return False

        # coordinate is out of right border
        if self.m_right >= 0:     
            if y - self.m_right * x - self.b_right < 0:
                return False

        elif y - self.m_right * x - self.b_right > 0:
            return False                    

        # coordinate is in play area
        return True   


    # checks if balloon is in borders
    def balloon_in_borders(self, image_3d: Image3D):
        return self.coordinat_in_borders(image_3d.phys_x_balloon, image_3d.phys_y_balloon)

    # checks if drone is in borders
    def drone_in_borders(self, image_3d: Image3D):
        return self.coordinat_in_borders(image_3d.phys_x_drone, image_3d.phys_y_drone)     

    
    def draw_borders(self, show_img, image_3d, color_in = (240,0,240), color_out = (240, 0, 240)):
        color = color_in
        if not self.balloon_in_borders(image_3d):
            color = color_out
        if self.set_borders:
            show_img = cv2.line(show_img, (self.pixels_coordinates[0][0], self.pixels_coordinates[0][1]), (self.pixels_coordinates[1][0], self.pixels_coordinates[1][1]), color, thickness=2)
            show_img = cv2.line(show_img, (self.pixels_coordinates[1][0], self.pixels_coordinates[1][1]), (self.pixels_coordinates[3][0], self.pixels_coordinates[3][1]), color, thickness=2)
            show_img = cv2.line(show_img, (self.pixels_coordinates[3][0], self.pixels_coordinates[3][1]), (self.pixels_coordinates[2][0], self.pixels_coordinates[2][1]), color, thickness=2)
            show_img = cv2.line(show_img, (self.pixels_coordinates[2][0], self.pixels_coordinates[2][1]), (self.pixels_coordinates[0][0], self.pixels_coordinates[0][1]), color, thickness=2)

        return show_img

    def coordinate_str(self, coordinate):
        return "%.2f,%.2f\n" % (coordinate[0], coordinate[1])

    def img_info_str(self):
        return "%.3f,%.3f,%.0f,%.0f\n" % (self.fov_horz, self.fov_vert, self.x_n_pix, self.z_n_pix)        

    def write_borders(self, filename):
        file_text = self.coordinate_str(self.coordinates[0]) + self.coordinate_str(self.coordinates[1]) \
             + self.coordinate_str(self.coordinates[2]) + self.coordinate_str(self.coordinates[3]) \
                + self.img_info_str()

        if os.path.exists(filename):
            os.remove(filename)
        with open(filename, 'w') as f:
            f.write(file_text)

        self.set_borders = True            
        print("Borders Saved")


    def read_coor(self, idx, line):
        line_s = line.split(',')
        self.coordinates[idx][0] = float(line_s[0])
        self.coordinates[idx][1] = float(line_s[1])


    def read_img_info(self, line):
        line_s = line.split(',')
        self.fov_horz = float(line_s[0])
        self.fov_vert = float(line_s[1])
        self.x_n_pix = int(line_s[2])
        self.z_n_pix = int(line_s[3])


    def read_borders(self, filename):
        if not os.path.exists(filename):
            print("ERROR: borders file does not exist")
            return
        
        with open(filename, 'r') as f:
            lines = f.readlines()

        self.read_coor(0, lines[0])
        self.read_coor(1, lines[1])
        self.read_coor(2, lines[2])
        self.read_coor(3, lines[3])  
        self.read_img_info(lines[4])      

        self.calc_borders()
        self.set_borders = True
        print("Borders Loaded")  

      