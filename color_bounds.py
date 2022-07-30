
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
            (self.lower[0], self.lower[1], self.lower[2], self.upper[0], self.upper[1], self.upper[2])

    def str_to_color_bound(self, lower, upper):
        lower = lower.split(',')
        upper = upper.split(',')
        self.lower = (int(lower[0]), int(lower[1]), int(lower[2]))
        self.upper = (int(upper[0]), int(upper[1]), int(upper[2]))
