from datetime import datetime

class LoopStatus:

    def __init__(self):
        self.tookoff = False
        self.start_track = False
        self.continue_loop = True
        self.hit = False
        self.hit_coords = 0
        self.hit_time = None

    def takeoff(self):
        self.tookoff = True

    def start_track(self):
        if self.tookoff:
            self.start_track = True

    def stop_track(self):
        self.start_track = False

    def stop_loop(self):
        self.continue_loop = False

    def reset(self):
        self.__init__()

    def hit_mode_on(self, coords):
        if self.start_track:
            self.hit = True
            self.hit_coords = coords
    
    def hit_mode_off(self):
        self.hit = False

    def set_hit_time(self):
        self.hit_time = datetime.now()

    def hit_mode(self):
        return self.hit


class Status(LoopStatus):
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            print('Creating the object')
            cls._instance = super(Status, cls).__new__(cls)
            # Put any initialization here.
        return cls._instance
