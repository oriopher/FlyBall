import datetime


class Frame:
    SEARCH_RANGE_SCALE_A = -0.25
    SEARCH_RANGE_SCALE_B = 140

    def __init__(self, image):
        """
        Initializes a frame captured from the camera.
        :param image: the image captured.
        """
        self.image = image
        self.threshold_size = image.shape[1] // 120
        self.capture_time = datetime.datetime.now()
        self.x_n_pix, self.z_n_pix = image.shape[1], image.shape[1]

    @staticmethod
    def search_range_scale(distance):
        """
        Calculates the scale of the search range in the frame, depending on the distance of an object from the camera.
        :param distance: the distance of the object that will be searched for.
        :return: the range of the search in each direction.
        """
        if distance < 50 or distance > 500:
            return 0
        search = Frame.SEARCH_RANGE_SCALE_A * distance + Frame.SEARCH_RANGE_SCALE_B
        return min(max(search, 40), 200)
