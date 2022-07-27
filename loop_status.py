from datetime import datetime

class LoopStatus:

    def __init__(self):
        self.tookoff_1 = False
        self.tookoff_2 = False
        self.start_1 = False
        self.start_2 = False
        self.first_seek_1 = False
        self.first_seek_2 = False
        self.continue_loop = True
        self.hit_1 = False
        self.hit_2 = False
        self.hit_coords = 0
        self.hit_time = None
        self.hit_height = 0
        self.prediction = 0 # 0 - disabled, 1 - starting, 2 - printing and testing predictions

    def takeoff_1(self):
        self.tookoff_1 = True

    def takeoff_2(self):
        self.tookoff_2 = True

    def start_track_1(self):
        if self.tookoff_1:
            self.start_1 = True
            if not self.first_seek_1:
                self.first_seek_1 = True

    def start_track_2(self):
        if self.tookoff_2:
            self.start_2 = True
            if not self.first_seek_2:
                self.first_seek_2 = True

    def stop_track_1(self):
        self.start_1 = False

    def stop_loop(self):
        self.continue_loop = False

    def reset(self):
        self.__init__()

    def hit_mode_on(self, coords):
        if self.start_1:
            self.hit_1 = True
            self.hit_coords = coords
    
    def hit_mode_off(self):
        self.hit_1 = False

    def set_hit_time(self):
        self.hit_time = datetime.now()

    def hit_mode(self):
        return self.hit_1

    def start_predictions(self):
        self.prediction = 1

    def test_predictions(self):
        self.prediction = 2

    def stop_predictions(self):
        self.prediction = 0

    def get_predict_stat(self):
        return self.prediction


class Status(LoopStatus):
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            print('Creating the object')
            cls._instance = super(Status, cls).__new__(cls)
            # Put any initialization here.
        return cls._instance
