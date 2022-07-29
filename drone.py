from djitellopy import Tello
from image_3d import Image3D
from loop_state_machine import ON_GROUND, STANDING_BY


class Drone:
    def __init__(self, ident: int, tello: Tello, middle: tuple[int, int]):
        self.tello = tello
        self.middle = middle
        self.id = ident
        self.x = self.y = self.z = self.vx = self.vy = self.vz = 0
        self.tookoff = self.start = self.first_seek = self.hit = False
        self.state = ON_GROUND()
        self.prediction = 0  # 0 - disabled, 1 - starting, 2 - printing and testing predictions
        self.x_0 = 0
        self.y_0 = 0
        self.dest_coords = (0, 0, 0)
        self.start_hit_timer = self.end_hit_timer = None
        self.drone_search_pred_coords = (0, 0, 0)
        self.drone_search_pred_time = 0
        self.testing = 0

    def connect(self):
        self.tello.connect()

    def save_coords(self, image_3d: Image3D):
        if self.id == 1:
            self.x = image_3d.phys_x_drone_1
            self.y = image_3d.phys_y_drone_1
            self.z = image_3d.phys_z_drone_1
            self.vx = image_3d.velocity_x_drone_1
            self.vy = image_3d.velocity_y_drone_1
            self.vz = image_3d.velocity_z_drone_1
        elif self.id == 2:
            self.y = image_3d.phys_y_drone_2
            self.z = image_3d.phys_z_drone_2
            self.vx = image_3d.velocity_x_drone_2
            self.vy = image_3d.velocity_y_drone_2
            self.vz = image_3d.velocity_z_drone_2

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


