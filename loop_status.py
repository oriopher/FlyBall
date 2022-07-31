from datetime import datetime

class LoopStatus:

    def __init__(self):
        self.prediction = 0 # 0 - disabled, 1 - starting, 2 - printing and testing predictions
        self.dest_coords = (0, 0, 0)
        self.testing = 0

    def reset(self):
        self.__init__()

    def ready_to_test(self):
        self.prediction = 4

    def start_predictions(self):
        self.prediction = 1

    def test_predictions(self):
        self.prediction = 2

    def stop_predictions(self):
        self.prediction = 0

    def get_predict_stat(self):
        return self.prediction

    def set_dest_coords(self, coords):
        self.dest_coords = coords

class Status(LoopStatus):
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            print('Creating the object')
            cls._instance = super(Status, cls).__new__(cls)
            # Put any initialization here.
        return cls._instance
