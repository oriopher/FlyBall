from turtle import left
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
        self.m_left, self.b_left = self.calc_linear_eq(self.images[3], self.images[1])
        self.m_right, self.b_right = self.calc_linear_eq(self.images[2], self.images[0])            
        self.y_upper_border = min(self.images[3].phys_y_balloon, self.images[2].phys_y_balloon)    
        self.y_low_border = max(self.images[1].phys_y_balloon, self.images[0].phys_y_balloon)

        # calculate the middle coordinates
        self.y_middle = (self.y_upper_border - self.y_upper_border) / 2
        x_left = (self.y_middle - self.b_left) / self.m_left
        x_right = (self.y_middle - self.b_right) / self.m_right
        self.x_middle = (x_right + x_left) / 2


    def calc_linear_eq(self, image_1: Image3D, image_2: Image3D):
        x2 = image_2.phys_x_balloon        
        y2 = image_2.phys_y_balloon
        x1 = image_1.phys_x_balloon        
        y1 = image_1.phys_y_balloon
        m = (y2 - y1) / (x2 - x1)
        b = y2 - m * x2  
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
            x4 = self.images[3].phys_x_balloon
            x3 = self.images[2].phys_x_balloon
            x2 = self.images[1].phys_x_balloon
            x1 = self.images[0].phys_x_balloon
            image = self.images[0].frame_left.image
        
            show_img = cv2.line(show_img, utils.phys_to_left_pix(x4, self.y_upper_border, test_2d.FLOOR_HEIGHT, image, left_cam), utils.phys_to_left_pix(x3, self.y_upper_border, test_2d.FLOOR_HEIGHT, image, left_cam), (0, 255, 0), thickness=2)
            show_img = cv2.line(show_img, utils.phys_to_left_pix(x2, self.y_low_border, test_2d.FLOOR_HEIGHT, image, left_cam), utils.phys_to_left_pix(x1, self.y_low_border, test_2d.FLOOR_HEIGHT, image, left_cam), (0, 255, 0), thickness=2)
            show_img = cv2.line(show_img, utils.phys_to_left_pix(x3, self.y_upper_border, test_2d.FLOOR_HEIGHT, image, left_cam), utils.phys_to_left_pix(x1, self.y_low_border, test_2d.FLOOR_HEIGHT, image, left_cam), (0, 255, 0), thickness=2)
            show_img = cv2.line(show_img, utils.phys_to_left_pix(x4, self.y_upper_border, test_2d.FLOOR_HEIGHT, image, left_cam), utils.phys_to_left_pix(x2, self.y_low_border, test_2d.FLOOR_HEIGHT, image, left_cam), (0, 255, 0), thickness=2)

        return show_img