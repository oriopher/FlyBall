from re import M
from turtle import update
from common import calc_linear_eq
from drone import Drone
from quadrangle import QUADRANGLE
import numpy as np

MARGINS = 25

class Obstacle:
    def __init__(self):
        #self.quad = QUADRANGLE()
        self.start = (0, 0)
        self.end = (0, 0)
        self.m_track = 0
        self.b_track = 0
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

    # update points and calculate linear equation between start point and end point
    def update_obstacle(self, drone: Drone):
        self.start = (drone.x, drone.y)
        self.end = (drone.dest_coords[0], drone.dest_coords[1])
        self.update_edges()
        return True


    def update_edges(self):
        self.b_low = self.b_track - MARGINS * np.sqrt(self.m_track ** 2 + 1)
        self.b_upper = self.b_track + MARGINS * np.sqrt(self.m_track ** 2 + 1)

        x_middle = (self.start[0] + self.end[0]) / 2
        y_middle = (self.start[1] + self.end[1]) / 2

        if self.m_track != 0:
            m_tmp = 1 / self.m_track
            b_tmp = y_middle - (1 / m_tmp) * x_middle
            x_middle_low = (self.b_low - self.b_track) / (self.m_track - m_tmp)
            y_middle_low = self.m_track * x_middle_low + self.b_low
            x_middle_upper = (self.b_upper - self.b_track) / (self.m_track - m_tmp)
            y_middle_upper = self.m_track * x_middle_upper + self.b_upper         

        self.set_coordinates(x_middle_low, y_middle_low, x_middle_upper, y_middle_upper)


    def set_coordinates(self, x_middle_low, y_middle_low, x_middle_upper, y_middle_upper):
        length = np.sqrt((self.start[0] - self.end[0]) ** 2 + (self.start[1] - self.end[1]) ** 2) +  2 * MARGINS


    def printCorners(x_middle_low, y_middle_low, x_middle_upper, y_middle_upper, length):
                
        # Horizontal rectangle
        if (x_middle_upper == x_middle_low):
            a_x = x_middle_upper - (length / 2)
            a_y = y_middle_upper
            
            d_x = x_middle_upper + (length / 2)
            d_y = y_middle_upper
            
            b.x = q.x - (length / 2.0)
            b.y = q.y
            
            c.x = q.x + (length / 2.0)
            c.y = q.y
            
        # Vertical rectangle
        elif (x_middle_low.y == q.y):
            a.y = x_middle_low.y - (length / 2.0)
            a.x = x_middle_low.x
            
            d.y = x_middle_low.y + (length / 2.0)
            d.x = x_middle_low.x
            
            b.y = q.y - (length / 2.0)
            b.x = q.x
            
            c.y = q.y + (length / 2.0)
            c.x = q.x
        
        # Slanted rectangle
        else:
            # Calculate slope of the side
            m = (x_middle_low.x - q.x) / (q.y - x_middle_low.y)
            
            # Calculate displacements along axes
            dx = (length / math.sqrt(1 + (m * m))) * 0.5
            dy = m * dx
            
            a.x = int(x_middle_low.x - dx)
            a.y =int(x_middle_low.y - dy)
            
            d.x = int(x_middle_low.x + dx)
            d.y = int(x_middle_low.y + dy)
            
            b.x = int(q.x - dx)
            b.y = int(q.y - dy)
            
            c.x = int(q.x + dx)
            c.y = int(q.y + dy)
            
        print(int(a.x), ", ", int(a.y), sep = "")
        print(int(b.x), ", ", int(b.y), sep = "")
        print(int(c.x), ", ", int(c.y), sep = "")
        print(int(d.x), ", ", int(d.y), sep = "")

        xy_display = cv2.line(xy_display, (a.x, a.y), (b.x, b.y), (255, 255, 255), 3)
        xy_display = cv2.line(xy_display, (a.x, a.y), (d.x, d.y) , (255, 255, 255), 3)
        xy_display = cv2.line(xy_display, (b.x, b.y), (c.x, c.y) , (255, 255, 255), 3)
        xy_display = cv2.line(xy_display, (c.x, c.y), (d.x, d.y) , (255, 255, 255), 3)

        return xy_display
        



        # checks if the drone is inside the obstacle
        def inside_obstacle(self, x, y):
            if self.start[0] <= self.end[0]:
                if  self.start[0] - MARGINS < x < self.end[0] + MARGINS:
                    distance = abs(self.m * x - y +self.b) / np.sqrt(self.m ** 2 + 1)
                    if distance < MARGINS:
                        return True
                    else:
                        return False

            if distance < MARGINS:
                return True

            return False


    # draws the obstacle on xy display frame
    def draw_obstacle(self):
        return True    
