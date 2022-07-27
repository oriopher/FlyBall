import os

class ColorBound:
    NO_LOWER_BOUNDS = (0, 0, 0)
    NO_UPPER_BOUNDS = (255, 255, 255)

    def __init__(self):
        self.lower = ColorBound.NO_LOWER_BOUNDS
        self.upper = ColorBound.NO_UPPER_BOUNDS

    def change(self, lower, upper):
        self.lower = lower
        self.upper = upper

    def __str__(self):
        return "%.0f,%.0f,%.0f\n%.0f,%.0f,%.0f\n" % \
            (self.lower[0], self.lower[1], self.lower[2],self.upper[0], self.upper[1], self.upper[2])

    def str_to_color_bound(self, lower, upper):
        lower = lower.split(',')
        upper = upper.split(',')
        self.lower = (int(lower[0]), int(lower[1]), int(lower[2]))
        self.upper = (int(upper[0]), int(upper[1]), int(upper[2]))


class ColorBounds:

    def __init__(self):
        self.ball_left = ColorBound()
        self.ball_right = ColorBound()
        self.drone_1_left = ColorBound()
        self.drone_1_right = ColorBound()
        self.drone_2_left = ColorBound()
        self.drone_2_right = ColorBound()

    def change_ball_left(self, lower, upper):
        self.ball_left.change(lower, upper)

    def change_ball_right(self, lower, upper):
        self.ball_right.change(lower, upper)

    def change_drone_1_left(self, lower, upper):
        self.drone_1_left.change(lower, upper)

    def change_drone_1_right(self, lower, upper):
        self.drone_1_right.change(lower, upper)

    def change_drone_2_left(self, lower, upper):
        self.drone_2_left.change(lower, upper)

    def change_drone_2_right(self, lower, upper):
        self.drone_2_right.change(lower, upper)

    def write_colors(self, filename):
        file_text = str(self.ball_left) + str(self.ball_right) + str(self.drone_1_left) + str(self.drone_1_right) +\
            str(self.drone_2_left) + str(self.drone_2_right)

        if os.path.exists(filename):
            os.remove(filename)
        with open(filename, 'w') as f:
            f.write(file_text)
            print("Colors Saved")

    def read_colors(self, filename):
        if not os.path.exists(filename):
            print("ERROR: colors file does not exist")
            return
        
        with open(filename, 'r') as f:
            lines = f.readlines()

        self.ball_left.str_to_color_bound(lines[0], lines[1])
        self.ball_right.str_to_color_bound(lines[2], lines[3])
        self.drone_1_left.str_to_color_bound(lines[4], lines[5])
        self.drone_1_right.str_to_color_bound(lines[6], lines[7])
        self.drone_2_left.str_to_color_bound(lines[8], lines[9])
        self.drone_2_right.str_to_color_bound(lines[10], lines[11])
        print("Colors Loaded")