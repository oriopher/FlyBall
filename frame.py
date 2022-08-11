import datetime


class Frame:
    SEARCH_RANGE_SCALE_A = -0.25
    SEARCH_RANGE_SCALE_B = 140

    def __init__(self, image):
        self.image = image
        self.threshold_size = image.shape[1] // 100
        self.capture_time = datetime.datetime.now()
        self.x_n_pix, self.z_n_pix = image.shape[1], image.shape[0]

    @staticmethod
    def search_range_scale(distance):
        if distance < 50 or distance > 500:
            return 0
        search = Frame.SEARCH_RANGE_SCALE_A * distance + Frame.SEARCH_RANGE_SCALE_B
        return min(max(search, 40), 200)
