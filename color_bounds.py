
class ColorBound:
    NO_LOWER_BOUNDS = (0, 0, 0)
    NO_UPPER_BOUNDS = (255, 255, 255)

    def __init__(self):
        self.lower = ColorBound.NO_LOWER_BOUNDS
        self.upper = ColorBound.NO_UPPER_BOUNDS

    def change(self, lower, upper):
        self.lower = lower
        self.upper = upper


class ColorBounds:

    def __init__(self):
        self.ball_left = ColorBound()
        self.ball_right = ColorBound()
        self.drone_left = ColorBound()
        self.drone_right = ColorBound()

    def change_ball_left(self, lower, upper):
        self.ball_left.change(lower, upper)

    def change_ball_right(self, lower, upper):
        self.ball_right.change(lower, upper)

    def change_drone_left(self, lower, upper):
        self.drone_left.change(lower, upper)

    def change_drone_right(self, lower, upper):
        self.drone_right.change(lower, upper)

