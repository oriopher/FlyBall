from common import calc_linear_eq
import numpy as np

MARGINS = 25

class Obstacle:
    def __init__(self):
        self.start = (0, 0)
        self.end = (0, 0)
        self.m = 0
        self.b = 0

    # calc linear equation between start point and end point
    def update_obstacle(self):
        self.m, self.b = calc_linear_eq(self.start, self.end)
        return True

    # checks if the drone is inside the obstacle
    def inside_obstacle(self, x, y, z):
        distance = abs(self.m * x - y +self.b) / np.sqrt(m ** 2 + 1)

        if distance < MARGINS:
            return True
            
        return False

    # sends drone out of the obstacle
    def exit_obstacle(self):
        return True    

    # draws the obstacle on xy display frame
    def draw_obstacle(self):
        return True    
