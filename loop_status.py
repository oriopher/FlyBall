
class LoopStatus:

    def __init__(self):
        self.tookoff = False
        self.start_track = False
        self.continue_loop = True

    def takeoff(self):
        self.tookoff = True

    def start(self):
        if self.tookoff:
            self.start_track = True

    def stop_loop(self):
        self.continue_loop = False

    def reset(self):
        self.__init__()


class Status(LoopStatus):
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            print('Creating the object')
            cls._instance = super(Status, cls).__new__(cls)
            # Put any initialization here.
        return cls._instance
