from datetime import datetime

from loop_state_machine import ON_GROUND, STANDING_BY


class LoopStatus:

    def __init__(self):
        self.tookoff = False
        self.start = False
        self.first_seek = False
        self.continue_loop = True
        self.hit = False
        self.state = ON_GROUND()
        self.prediction = 0 # 0 - disabled, 1 - starting, 2 - printing and testing predictions
        self.x_0 = 0
        self.y_0 = 0
        self.dest_coords = (0,0,0)
        self.start_hit_timer = None
        self.end_hit_timer = None
        self.drone_search_pred_coords = (0,0,0)
        self.drone_search_pred_time = 0

    def takeoff(self):
        self.tookoff = True

    def start_track(self, x_0=0, y_0=0):
        if self.tookoff:
            self.start = True
            if not self.first_seek:
                self.first_seek = True
                self.x_0 = x_0
                self.y_0 = y_0

    def stop_track(self):
        if self.start:
            self.start = False

    def stop_hit(self):
        if self.hit:
            self.hit = False
            self.state = STANDING_BY()

    def stop_loop(self):
        self.continue_loop = False

    def reset(self):
        self.__init__()

    def hit_mode_on(self):
        if self.start:
            self.hit = True
            self.start_hit_timer = datetime.now()
    
    def hit_mode_off(self):
        self.hit = False
        self.end_hit_timer = datetime.now()

    def hit_mode(self):
        return self.hit

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
