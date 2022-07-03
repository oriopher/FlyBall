from image_3d import Image3D

class Borders:

    def __init__(self):
        self.index = 0
        self.images = [None] * 4       # (x_4, y_4) ################################ (x_3, y_3)
        self.M_LEFT = 0                               ############################
        self.B_LEFT = 0                                 #######################
        self.M_RIGHT = 0                                  ##################
        self.B_RIGHT = 0                                    ##############
        self.Y_UPPER_BORDER = 0                  # (x_2, y_2) ########## (x_1, y_1)
        self.Y_LOW_BORDER = 0
        self.X_MIDDLE = 0
        self.Y_MIDDLE = 0


    # saving the image in the array
    def set_image(self, image_3d: Image3D):
        if (self.index <= 3):
            self.images[self.index] = image_3d
            self.index += 1
        if self.index > 4:
            self.calc_left_border()
            self.calc_right_border()
            self.set_y_borders()    


    def calc_left_border(self):
        x_4 = self.images[3].phys_x_balloon        
        y_4 = self.images[3].phys_y_balloon
        x_2 = self.images[1].phys_x_balloon        
        y_2 = self.images[1].phys_y_balloon
        self.M_LEFT = (y_4 - y_2) / (x_4 - x_2)
        self.B_LEFT = y_4 - self.M_LEFT * x_4


    def calc_right_border(self):
        x_3 = self.images[2].phys_x_balloon        
        y_3 = self.images[2].phys_y_balloon
        x_1 = self.images[0].phys_x_balloon        
        y_1 = self.images[0].phys_y_balloon
        self.M_RIGHT = (y_3 - y_1) / (x_3 - x_1)
        self.B_RIGHT = y_3 - self.M_RIGHT * x_3


    def set_y_borders(self):
        self.Y_LOW_BORDER = self.images[0].phys_y_balloon
        self.Y_UPPER_BORDER = self.images[2].phys_y_balloon

        # calculate the middle coordinates
        self.Y_MIDDLE = (self.Y_UPPER_BORDER - self.Y_LOW_BORDER) / 2
        x_left = (self.Y_MIDDLE - self.B_LEFT) / self.M_LEFT
        x_right = (self.Y_MIDDLE - self.B_RIGHT) / self.M_RIGHT
        self.X_MIDDLE = (x_right + x_left) / 2


    # checks if balloon is in borders
    def in_borders(self, image_3d: Image3D):

        # balloon is too far or too close to camera
        if (image_3d.phys_y_balloon < self.Y_LOW_BORDER or image_3d.phys_y_balloon > self.Y_UPPER_BORDER):
            return 1

        # ballon is out of left border
        if (image_3d.phys_y_balloon - self.M_LEFT * image_3d.phys_x_balloon - self.B_LEFT < 0):
            return 1

        # ballon is out of right border
        if (image_3d.phys_y_balloon - self.M_RIGHT * image_3d.phys_x_balloon - self.B_RIGHT > 0):
            return 1

        # balloon is in play area
        return 0    


