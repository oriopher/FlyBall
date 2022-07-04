import os
from image_3d import Image3D
import test_2d
import camera
import utils
import cv2

class Borders:
    def __init__(self):
        self.index = 0
        self.images = [None] * 4       # (x_4, y_4) ################################ (x_3, y_3)
        self.m_left = 0                               ############################
        self.b_left = 0                                 #######################
        self.m_right = 0                                  ##################
        self.b_right = 0                                    ##############
        self.y_upper_border = 0                  # (x_2, y_2) ########## (x_1, y_1)
        self.y_upper_border = 0
        self.x_middle = 0
        self.y_middle = 0
        self.coor1 = [0, 0]
        self.coor2 = [0, 0]
        self.coor3 = [0, 0]
        self.coor4 = [0, 0]

        self.set_borders = False


    # saving the image in the array
    def set_image(self, image_3d: Image3D):
        if (self.index <= 3):
            self.images[self.index] = image_3d
            self.index += 1
        if self.index > 4:
            self.calc_borders()
            self.set_borders = True


    def calc_borders(self):
        self.coor1 = [self.images[0].phys_x_balloon, self.images[0].phys_y_balloon]
        self.coor2 = [self.images[1].phys_x_balloon, self.images[1].phys_y_balloon]
        self.coor3 = [self.images[2].phys_x_balloon, self.images[2].phys_y_balloon]
        self.coor4 = [self.images[3].phys_x_balloon, self.images[3].phys_y_balloon]

        self.m_left, self.b_left = self.calc_linear_eq(self.coor4, self.coor2)
        self.m_right, self.b_right = self.calc_linear_eq(self.coor3, self.coor1)            
        self.y_upper_border = min(self.coor3[1], self.coor4[1])    
        self.y_low_border = max(self.coor1[1], self.coor2[1])

        # calculate the middle coordinates
        self.y_middle = (self.y_upper_border - self.y_upper_border) / 2
        x_left = (self.y_middle - self.b_left) / self.m_left
        x_right = (self.y_middle - self.b_right) / self.m_right
        self.x_middle = (x_right + x_left) / 2


    def calc_linear_eq(self, coor1, coor2):
        m = (coor2[1] - coor1[1]) / (coor2[0] - coor1[0])
        b = coor2[1] - m * coor2[0]  
        return m, b


    # checks if balloon is in borders
    def balloon_in_borders(self, image_3d: Image3D):

        # balloon is too far or too close to camera
        if (image_3d.phys_y_balloon < self.y_upper_border or image_3d.phys_y_balloon > self.y_upper_border):
            return False

        # ballon is out of left border
        if (image_3d.phys_y_balloon - self.m_left * image_3d.phys_x_balloon - self.b_left < 0):
            return False

        # ballon is out of right border
        if (image_3d.phys_y_balloon - self.m_right * image_3d.phys_x_balloon - self.b_right > 0):
            return False

        # balloon is in play area
        return True   

    
    def draw_borders(self, show_img, left_cam: camera):
        if self.set_borders:
            image = self.images[0].frame_left.image
            show_img = cv2.line(show_img, utils.phys_to_left_pix(self.coor4[0], self.y_upper_border, test_2d.FLOOR_HEIGHT, image, left_cam), utils.phys_to_left_pix(self.coor3[0], self.y_upper_border, test_2d.FLOOR_HEIGHT, image, left_cam), (0, 255, 0), thickness=2)
            show_img = cv2.line(show_img, utils.phys_to_left_pix(self.coor2[0], self.y_low_border, test_2d.FLOOR_HEIGHT, image, left_cam), utils.phys_to_left_pix(self.coor1[0], self.y_low_border, test_2d.FLOOR_HEIGHT, image, left_cam), (0, 255, 0), thickness=2)
            show_img = cv2.line(show_img, utils.phys_to_left_pix(self.coor3[0], self.y_upper_border, test_2d.FLOOR_HEIGHT, image, left_cam), utils.phys_to_left_pix(self.coor1[0], self.y_low_border, test_2d.FLOOR_HEIGHT, image, left_cam), (0, 255, 0), thickness=2)
            show_img = cv2.line(show_img, utils.phys_to_left_pix(self.coor4[0], self.y_upper_border, test_2d.FLOOR_HEIGHT, image, left_cam), utils.phys_to_left_pix(self.coor2[0], self.y_low_border, test_2d.FLOOR_HEIGHT, image, left_cam), (0, 255, 0), thickness=2)

        return show_img


    def write_borders(self, filename):
        file_text = str(self.m_left) + str(self.b_left) + str(self.m_right) + str(self.b_right) + str(self.y_low_border) + str(self.y_upper_border) + str(self.y_middle) + str(self.x_middle) + str(self.coor1[0]) + str(self.coor1[1]) + str(self.coor2[0]) + str(self.coor2[1]) + str(self.coor3[0]) + str(self.coor3[1]) + str(self.coor4[0]) + str(self.coor4[1])

        if os.path.exists(filename):
            os.remove(filename)
        with open(filename, 'w') as f:
            f.write(file_text)
            print("Borders Saved")


    def read_borders(self, filename):
        if not os.path.exists(filename):
            print("ERROR: borders file does not exist")
            return
        
        with open(filename, 'r') as f:
            lines = f.readlines()

        self.m_left = float(lines[0].split(','))
        self.b_left = float(lines[1].split(','))
        self.m_right = float(lines[2].split(','))
        self.b_right = float(lines[3].split(','))
        self.y_low_border = float(lines[4].split(','))
        self.y_upper_border = float(lines[5].split(','))
        self.y_middle = float(lines[6].split(','))
        self.x_middle = float(lines[7].split(','))
        self.coor1[0] = float(lines[8].split(','))
        self.coor1[1] = float(lines[9].split(','))
        self.coor2[0] = float(lines[10].split(','))
        self.coor2[1] = float(lines[11].split(','))
        self.coor3[0] = float(lines[12].split(','))
        self.coor3[1] = float(lines[13].split(','))
        self.coor4[0] = float(lines[14].split(','))
        self.coor4[1] = float(lines[5].split(','))

        print("Borders Loaded")    