
class LoopStatus:

    def __init__(self):
        self.tookoff = False
        self.start_track = False
        self.continue_loop = True
        self.hit = False
        self.hit_height = 0

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

    def hit_mode_on(self, height):
        if self.start_track:
            self.hit = True
            self.hit_height = height
    
    def hit_mode_off(self):
        self.hit = False

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
