from datetime import datetime

class LoopStatus:

    def __init__(self):
        self.tookoff = False
        self.start = False
        self.first_seek = False
        self.continue_loop = True
        self.hit = False
        self.hit_coords = 0
        self.hit_time = None
        self.hit_height = 0
        self.prediction = 0 # 0 - disabled, 1 - starting, 2 - printing and testing predictions

    def takeoff(self):
        self.tookoff = True

    def start_track(self):
        if self.tookoff:
            self.start = True
            if not self.first_seek:
                self.first_seek = True

    def stop_track(self):
        self.start = False

    def stop_loop(self):
        self.continue_loop = False

    def reset(self):
        self.__init__()

    def hit_mode_on(self, coords):
        if self.start:
            self.hit = True
            self.hit_coords = coords
    
    def hit_mode_off(self):
        self.hit = False

    def set_hit_time(self):
        self.hit_time = datetime.now()

    def hit_mode(self):
        return self.hit

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
