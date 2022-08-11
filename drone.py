from datetime import datetime
import numpy as np

from consts import DRONE_DEFAULT_HEIGHT
from obstacle import Obstacle
from recognizable_object import RecognizableObject
# from loop_state_machine_human_drone import ON_GROUND
# from loop_state_machine_passive_test import ON_GROUND
from loop_state_machine_2_drones_volleyball import ON_GROUND
from tello_drone_control import TelloDroneControl


class Drone:
    OLD_DEST_NUM = 4

    def __init__(self, ident: int, text_colors,
                 iface_ip: str = '192.168.10.2'):
        self.recognizable_object = RecognizableObject(text_colors, "drone" + str(ident))
        self.drone_control = TelloDroneControl(iface_ip)
        self.home = (0, 0)
        self.ident = ident
        self.tookoff = self.start = self.active = False
        self.state = ON_GROUND()
        self.x_0 = 0
        self.y_0 = 0
        self.dest_coords = (0, 0, 0)
        self.old_dest_coords = None
        self.start_hit_timer = None
        self.drone_search_pred_coords = (0, 0, 0)
        self.drone_search_pred_time = 0
        self.testing = 0
        self.active = False
        self.default_height = DRONE_DEFAULT_HEIGHT
        self.obstacle = None
        self.start_hit_vx = 0
        self.start_hit_vy = 0

    @property
    def x(self):
        return self.recognizable_object.x

    @property
    def y(self):
        return self.recognizable_object.y

    @property
    def z(self):
        return self.recognizable_object.z

    @property
    def vx(self):
        return self.recognizable_object.vx

    @property
    def vy(self):
        return self.recognizable_object.vy

    @property
    def vz(self):
        return self.recognizable_object.vz

    @property
    def battery(self):
        return "drone{} battery = {:d}%".format(self.ident, self.drone_control.get_battery())

    def detect_color(self, is_left):
        self.recognizable_object.detect_color(is_left)

    def takeoff(self):
        self.drone_control.connect()
        print(self.battery)
        self.drone_control.takeoff()

    def land(self):
        self.drone_control.land()
        self.tookoff = False
        print(self.battery)

    def start_track(self):
        if self.tookoff:
            self.start = True
            self.x_0 = self.x
            self.y_0 = self.y

    def stop_track(self):
        if self.start:
            self.start = False

    def search_pred_start(self):
        self.drone_search_pred_time = datetime.now()
        self.drone_search_pred_coords = (self.x, self.y, self.z)
        self.old_dest_coords = np.zeros((0, 3))
        
    def start_hit(self):
        self.start_hit_timer = datetime.now()
        self.start_hit_vx = self.vx
        self.start_hit_vy = self.vy
        print("hit v0: (%.0f, %.0f)" % (self.vx, self.vy))

    def set_home(self, coords):
        self.home = coords

    def new_pred(self, coords):
        if len(self.old_dest_coords) >= self.OLD_DEST_NUM:
            self.old_dest_coords = self.old_dest_coords[1:]
        self.old_dest_coords = np.vstack([self.old_dest_coords, coords])
        return np.mean(self.old_dest_coords, axis=0)

    def track_3d(self, dest_x, dest_y, dest_z, obstacle=None):
        if not self.active and obstacle:
            # print('original dest: ', dest_x, dest_y)
            dest_x, dest_y = obstacle.bypass_obstacle_coordinates((self.x, self.y),(dest_x, dest_y))
            # print("dest: ", dest_x, dest_y)
            # print("location: ", self.x, self.y)
            # print(obstacle.coordinates)
        if self.dest_coords != (0,0,0):
            self.drone_control.track_3d(dest_x, dest_y, dest_z, self.recognizable_object)
        self.dest_coords = (dest_x, dest_y, dest_z)

    def track_balloon(self, balloon, obstacle=None):
        self.track_2d(balloon.x, balloon.y, obstacle)

    def track_2d(self, dest_x, dest_y, obstacle=None):
        self.track_3d(dest_x, dest_y, self.default_height, obstacle)

    def go_home(self, obstacle=None):
        self.track_2d(*self.home, obstacle)

    def track_hitting(self, dest_x, dest_y, dest_z):
        self.dest_coords = (dest_x, dest_y, dest_z)
        self.drone_control.track_hitting(dest_x, dest_y, dest_z, self.recognizable_object)

    def track_hitting2(self, dest_x, dest_y, dest_z):
        self.dest_coords = (dest_x, dest_y, dest_z)
        self.drone_control.track_hitting2(dest_x, dest_y, dest_z, self.recognizable_object, self.start_hit_vx, self.start_hit_vy)

    def track_descending(self, obstacle=None):
        dest_x, dest_y = self.home
        if not self.active and obstacle:
            dest_x, dest_y = obstacle.bypass_obstacle_coordinates((self.x, self.y),(dest_x, dest_y))
        self.drone_control.track_descending(dest_x, dest_y, self.recognizable_object)
        self.dest_coords = (dest_x, dest_y, self.default_height)

    def track_descending_2drones(self, other_drone):
        SPEED_FACTOR = 100
        vec = [self.x - other_drone.x, self.y - other_drone.y] # vector drone minus vector other_drone
        abs_vector = np.sqrt(vec[0]**2 + vec[1]**2)
        vec[0] = SPEED_FACTOR * vec[0] / abs_vector
        vec[1] =  SPEED_FACTOR * vec[1] / abs_vector 
        dest_x = self.x + vec[0]
        dest_y = self.y + vec[1]
        self.drone_control.track_descending(dest_x, dest_y, self.recognizable_object)
        self.dest_coords = (dest_x, dest_y, self.default_height)
        

    def stop(self):
        self.drone_control.stop()

    def set_obstacle(self, left_cam):
        if self.dest_coords == (0, 0, 0):
            self.obstacle = None
            return
        self.obstacle = Obstacle(self, left_cam)
 