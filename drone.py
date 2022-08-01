import datetime
import numpy as np

from common import DRONE_DEFAULT_HEIGHT
from recognizable_object import RecognizableObject
from loop_state_machine import ON_GROUND, STANDING_BY
from tello import Tello


class Drone(RecognizableObject):
    OLD_DEST_NUM = 2

    def __init__(self, ident: int, text_colors: tuple[int, int, int], radius: int ,middle: tuple[int, int] = (0, 0),
                 iface_ip: str = '192.168.10.2'):
        super().__init__(text_colors, radius, "drone" + str(ident))
        self.iface_ip = iface_ip
        self.tello = None
        self.middle = middle
        self.id = ident
        self.tookoff = self.start = self.first_seek = False
        self.state = ON_GROUND()
        self.x_0 = 0
        self.y_0 = 0
        self.dest_coords = (0, 0, 0)
        self.old_dest_coords = None
        self.start_hit_timer = self.end_hit_timer = None
        self.drone_search_pred_coords = (0, 0, 0)
        self.drone_search_pred_time = 0
        self.testing = 0

    def battery_status(self, to_print):
        if to_print:
            print("battery = ", self.tello.get_battery(), "%")

    def takeoff(self, battery=True):
        if not self.tello:
            self.tello = Tello(iface_ip=self.iface_ip)
        self.tello.connect()
        self.battery_status(battery)
        self.tello.takeoff()
        self.tookoff = True

    def land(self, battery=True):
        self.tello.land()
        self.battery_status(battery)

    def send_rc_control(self, left_right_velocity: int, forward_backward_velocity: int, up_down_velocity: int,
                        yaw_velocity: int):
        self.tello.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)

    def wait_rc_control(self):
        while not self.tello.send_rc_control:
            continue
        return

    def get_battery(self):
        return self.tello.get_battery()

    def start_track(self):
        if self.tookoff:
            self.start = True
            if not self.first_seek:
                self.first_seek = True
                self.x_0 = self.x
                self.y_0 = self.y

    def stop_track(self):
        if self.start:
            self.start = False

    def stop_hit(self):
        if self.hit:
            self.hit = False
            self.state = STANDING_BY()

    def set_dest_coords(self, coords: tuple[float, float, float]):
        self.dest_coords = coords

    def search_pred_start(self):
        self.drone_search_pred_time = datetime.datetime.now()
        self.drone_search_pred_coords = (self.x, self.y, self.z)
        self.old_dest_coords = np.zeros((0, 3))
        
    def start_hit(self):
        self.start_hit_timer = datetime.datetime.now()

    def set_middle(self, middle: tuple[float, float]):
        self.middle = middle

    def new_pred(self, coords):
        if len(self.old_dest_coords) >= self.OLD_DEST_NUM:
            self.old_dest_coords = self.old_dest_coords[1:]
        self.old_dest_coords = np.vstack([self.old_dest_coords, coords])
        return np.mean(self.old_dest_coords, axis=0)

    def track_3d(self, dest_x: float, dest_y: float, dest_z: float):
        x_cm_rel = dest_x - self.x
        y_cm_rel = dest_y - self.y
        z_cm_rel = dest_z - self.z
        self.set_dest_coords((dest_x, dest_y, dest_z))

        left_right = self.velocity_control_function(x_cm_rel, self.vx, 'x')
        for_back = self.velocity_control_function(y_cm_rel, self.vy, 'y')
        up_down = self.velocity_control_function(z_cm_rel, self.vz, 'z')
        if self.tello.send_rc_control:
            self.send_rc_control(left_right, for_back, up_down, 0)

    def track_balloon(self, balloon: RecognizableObject):
        self.track_2d(balloon.x, balloon.y)

    def seek_middle(self):
        self.track_2d(*self.middle)

    def track_2d(self, dest_x: int, dest_y: int):
        self.track_3d(dest_x, dest_y, DRONE_DEFAULT_HEIGHT)

    @staticmethod
    def velocity_control_function(cm_rel, real_velocity, direction):
        # this function assumes the drone is looking at the cameras.

        # These parameters achieve a hit within 2-3 seconds using lin + sqrt.
        # there is a plot of this function in the documentation.
        # MIN_VEL = 9 # under this speed the tello receives this as 0
        # LOWER_BOUND = 5 # when the drone is closer than this we will just let it stop
        # if direction == 'z':
        #     MAX_VEL = 30
        #     A_SQRT = 2
        #     A_LINEAR = 0.9
        #     B = 1.2
        #     STOPPING_VEL = 0
        #     C = 0
        # elif direction == 'x' or direction == 'y':
        #     STOPPING_VEL = 20
        #     MAX_VEL = 60
        #     A_SQRT = 3
        #     A_LINEAR = 1
        #     B = 0.7
        #     C = 5
        # else:
        #     return 0

        MIN_VEL = 9  # under this speed the tello receives this as 0
        LOWER_BOUND = 5  # when the drone is closer than this we will just let it stop

        STOPPING_VEL = 30  # The velocity applied in order to stop faster
        MAX_VEL = 70
        A_SQRT = 3
        A_LINEAR = 1
        C = 5
        B = 0.6  # Associated with the stopping distance

        if direction == 'z':
            STOPPING_VEL = 0
            B = 0.3
            A_LINEAR = 1.5
            LOWER_BOUND = 2

        limit = max(B * abs(real_velocity), LOWER_BOUND)

        if abs(cm_rel) < limit:
            if abs(cm_rel) < LOWER_BOUND:
                velocity = 0
            else:
                velocity = np.sign(real_velocity) * STOPPING_VEL

        else:
            # velocity_pot = int(min(A * (abs(cm_rel) - limit) + MIN_VEL, MAX_VEL))
            velocity_pot = int(
                min(max(A_SQRT * np.sqrt(abs(cm_rel) - limit), A_LINEAR * (abs(cm_rel) - limit) - C) + MIN_VEL,
                    MAX_VEL))
            velocity = -np.sign(cm_rel) * velocity_pot

        if direction == 'x':
            return int(velocity)
        elif direction == 'y':
            return int(velocity)
        elif direction == 'z':
            return -int(velocity)

    def track_hitting(self, dest_x: float, dest_y: float, dest_z: float):
        self.set_dest_coords((dest_x, dest_y, dest_z))
        rx = dest_x - self.x
        ry = dest_y - self.y
        rz = dest_z - self.z
        r = np.sqrt(rx ** 2 + ry ** 2 + rz ** 2)
        theta = np.arccos(rz / r)
        phi = np.arctan2(ry, rx)
        MAX_VEL = 100
        MIN_VEL = 9
        A = 1
        vz = MAX_VEL
        vx = min(int(A * (vz-MIN_VEL) * np.tan(theta) * np.cos(phi)) + MIN_VEL, MAX_VEL)
        vy = min(int(A * (vz-MIN_VEL) * np.tan(theta) * np.sin(phi)) + MIN_VEL, MAX_VEL)

        left_right, for_back, up_down = -vx, -vy, vz
        if self.tello.send_rc_control:
            self.send_rc_control(left_right, for_back, up_down, 0)
