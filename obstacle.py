from quadrangle import QUADRANGLE
import numpy as np
import cv2

MARGINS = 25

class Obstacle:
    def __init__(self):
        self.quad = QUADRANGLE()
        self.start = (0, 0)
        self.end = (0, 0)
        self.m_track = 0
        self.b_track = 0  
        self.active = None     
        self.passive = None
        self.set_obstacle = False 


    """   def update_obstacle(self, other_drone: Drone):
        self.start = (other_drone.x, other_drone.y)
        self.end = (other_drone.dest_coords[0], other_drone.dest_coords[1])
        self.update_edges()
        self.passive = True
        """

    def update_start(self, x, y):
        self.start = (x, y)       
        self.update_edges()


    def update_obstacle(self, x, y, x_dest, y_dest, active_object, passive_object, left_cam):
        self.quad.fov_horz = left_cam.fov_horz
        self.quad.fov_vert = left_cam.fov_vert
        self.quad.x_n_pix = active_object.frame_left.image.shape[1]
        self.quad.z_n_pix = active_object.frame_left.image.shape[0]
        self.start = (x, y)
        self.end = (x_dest, y_dest)
        self.update_edges()
        self.active = active_object
        self.passive = passive_object
        self.set_obstacle = True



    def update_edges(self):
        # rectangle parallel to y axis
        if self.end[0] == self.start[0]: # maybe smaller than epsilon?
            x_middle_low = self.end[0] - MARGINS
            x_middle_upper = self.start[0] + MARGINS
            y_middle_low = y_middle
            y_middle_upper = y_middle
        
        else:
            self.m_track = (self.start[1] - self.end[1]) / (self.start[0] - self.end[0])
            self.b_low = self.b_track - MARGINS * np.sqrt(self.m_track ** 2 + 1)
            self.b_upper = self.b_track + MARGINS * np.sqrt(self.m_track ** 2 + 1)

            x_middle = (self.start[0] + self.end[0]) / 2
            y_middle = (self.start[1] + self.end[1]) / 2

            if self.m_track != 0:
                m_tmp = - 1 / self.m_track
                x_middle_low = (self.b_low - self.b_track) / (self.m_track - m_tmp)
                y_middle_low = self.m_track * x_middle_low + self.b_low
                x_middle_upper = (self.b_upper - self.b_track) / (self.m_track - m_tmp)
                y_middle_upper = self.m_track * x_middle_upper + self.b_upper         

            # drone track is parallal to x axis
            else:
                x_middle_low = x_middle
                x_middle_upper = x_middle
                y_middle_low = y_middle - MARGINS
                y_middle_upper = y_middle + MARGINS

        self.calc_corners(x_middle_low, y_middle_low, x_middle_upper, y_middle_upper)


    # finds the vertexes of the rectangle
    def calc_corners(self, x_middle_low, y_middle_low, x_middle_upper, y_middle_upper):
        length = np.sqrt((self.start[0] - self.end[0]) ** 2 + (self.start[1] - self.end[1]) ** 2) +  2 * MARGINS
                
        # Horizontal rectangle
        if x_middle_upper == x_middle_low:
            a_x = x_middle_upper - (length / 2)
            a_y = y_middle_upper
            
            d_x = x_middle_upper + (length / 2)
            d_y = y_middle_upper
            
            b_x = x_middle_low - (length / 2)
            b_y = y_middle_low
            
            c_x = x_middle_low + (length / 2)
            c_y = y_middle_low
            
        # Vertical rectangle
        elif y_middle_upper == y_middle_low:
            a_y = y_middle_upper - (length / 2)
            a_x = x_middle_upper
            
            d_y = y_middle_upper + (length / 2)
            d_x = x_middle_upper
            
            b_y = y_middle_low - (length / 2)
            b_x = x_middle_low
            
            c_y = y_middle_low + (length / 2)
            c_x = x_middle_low
        
        # Slanted rectangle
        else:
            # Calculate slope of the side
            m = (x_middle_upper - x_middle_low) / (y_middle_low - y_middle_upper)
            
            # Calculate displacements along axes
            dx = (length / np.sqrt(1 + (m ** 2))) * 0.5
            dy = m * dx
            
            a_x = x_middle_upper - dx
            a_y = y_middle_upper - dy
            
            d_x = x_middle_upper + dx
            d_y = y_middle_upper + dy
            
            b_x = x_middle_low - dx
            b_y = y_middle_low - dy
            
            c_x = x_middle_low + dx
            c_y = y_middle_low + dy

        # saves rectangle's coordinates    
        self.quad.coordinates[0] = [c_x, c_y]
        self.quad.coordinates[1] = [b_x, b_y]
        self.quad.coordinates[2] = [d_x, d_y]
        self.quad.coordinates[3] = [a_x, a_y]
        print(self.quad.coordinates)
        self.quad.calc_edges()

    
    def delete_obsticle(self):
        self.passive = False


    # checks if the drone is inside the obstacle
    def inside_obstacle(self):
        if not self.set_obstacle:
            return True
        # if rectangle is parallel to y axis
        if self.end[0] == self.start[0]:
            if self.end[0] - MARGINS < self.passive.x < self.end[0] + MARGINS:
                if np.min(self.end[1], self.start[1]) - MARGINS < self.passive.y < np.max(self.end[1], self.start[1]) + MARGINS:
                    return True   
            return False
        
        else:
            return self.quad.coordinate_in_quadrangle(self.passive.x, self.passive.y)    


    # draws the obstacle on left frame
    def draw_obstacle(self, show_img, recognizable_object, color_in=(240, 0, 0), color_out=(0, 240, 0)):
        color = color_out
    
        if self.passive == recognizable_object:
            if self.inside_obstacle():
                color = color_in
            show_img = cv2.line(show_img, self.quad.pixels_coordinates[0], self.quad.pixels_coordinates[1], color, 3)
            show_img = cv2.line(show_img, self.quad.pixels_coordinates[3], self.quad.pixels_coordinates[2], color, 3)
            show_img = cv2.line(show_img, self.quad.pixels_coordinates[3], self.quad.pixels_coordinates[1], color, 3)
            show_img = cv2.line(show_img, self.quad.pixels_coordinates[0], self.quad.pixels_coordinates[2], color, 3)            
            
        return show_img

