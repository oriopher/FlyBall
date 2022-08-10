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
    """
    A class for the drone.
    """
    # The number of old destinations in order to calculate a mean destination (prediction).
    OLD_DEST_NUM = 4

    def __init__(self, ident: int, text_color, radius,
                 iface_ip: str = '192.168.10.2'):
        """
        Initializes the drone.
        :param ident: a unique id number for the drone.
        :param text_color: the color that will be representing the drone in displays.
        :param radius: the radius of the drone.
        :param iface_ip: the ip of the interface that connects to the specific drone.
        """
        self.recognizable_object = RecognizableObject(text_color, radius, "drone" + str(ident))
        self.drone_control = TelloDroneControl(iface_ip)
        self.home = (0, 0)
        self.ident = ident
        self.tookoff = self.start = self.first_seek = self.active = False
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
        """
        :return: the x point of the drone.
        """
        return self.recognizable_object.x

    @property
    def y(self):
        """
        :return: the y point of the drone.
        """
        return self.recognizable_object.y

    @property
    def z(self):
        """
        :return: the z point of the drone.
        """
        return self.recognizable_object.z

    @property
    def vx(self):
        """
        :return: the drones velocity in the x axis.
        """
        return self.recognizable_object.vx

    @property
    def vy(self):
        """
        :return: the drones velocity in the y axis.
        """
        return self.recognizable_object.vy

    @property
    def vz(self):
        """
        :return: the drones velocity in the z axis.
        """
        return self.recognizable_object.vz

    @property
    def battery(self):
        """
        :return: a string showing the drones battery percentage.
        """
        return "drone{} battery = {:d}%".format(self.ident, self.drone_control.get_battery())

    def detect_color(self, is_left):
        """
        Detects the drones balloon colors in one camera.
        :param is_left: whether to detect in the left camera.
        """
        self.recognizable_object.detect_color(is_left)

    def takeoff(self):
        """
        Commands the drone to takeoff.
        """
        self.drone_control.connect()
        print(self.battery)
        self.drone_control.takeoff()

    def land(self):
        """
        Commands the drone to land.
        """
        self.drone_control.land()
        self.tookoff = False
        print(self.battery)

    def stop(self):
        """
        Commands the drone to stop.
        """
        self.drone_control.stop()

    def start_track(self):
        """
        Sets the drone to start tracking.
        If this is the first time the function sets the drone started to seek and saves it's initial x,y coordinates.
        """
        if self.tookoff:
            self.start = True
            if not self.first_seek:
                self.first_seek = True
                self.x_0 = self.x
                self.y_0 = self.y

    def stop_track(self):
        """
        Indicates to the drone to stop tracking.
        """
        if self.start:
            self.start = False

    def search_pred_start(self):
        """
        Resets the search prediction parameters of the drone.
        """
        self.drone_search_pred_time = datetime.now()
        self.drone_search_pred_coords = (self.x, self.y, self.z)
        self.old_dest_coords = np.zeros((0, 3))

    def start_hit(self):
        """
        Resets the hitting parameters of the drone.
        """
        self.start_hit_timer = datetime.now()
        self.start_hit_vx = self.vx
        self.start_hit_vy = self.vy

    def set_home(self, coords):
        """
        Sets the drones home location.
        :param coords: the coordinates of the home.
        """
        self.home = coords

    def new_pred(self, coords):
        """
        Calculates a new prediction for the drone to track using the old predictions.
        :param coords: the coordinates of a new prediction.
        :return: the average prediction.
        """
        if len(self.old_dest_coords) >= self.OLD_DEST_NUM:
            self.old_dest_coords = self.old_dest_coords[1:]
        self.old_dest_coords = np.vstack([self.old_dest_coords, coords])
        return np.mean(self.old_dest_coords, axis=0)

    def track_3d(self, dest_x, dest_y, dest_z, obstacle=None):
        """
        Moves the drone in the direction of the wanted destination.
        If the drone is not active and an obstacle is passed to the function - bypass it.
        :param dest_x: the desired x point.
        :param dest_y: the desired y point.
        :param dest_z: the desired z point
        :param obstacle: an Obstacle the drone should avoid if its not active.
        """
        if not self.active and obstacle:
            # print('original dest: ', dest_x, dest_y)
            dest_x, dest_y = obstacle.bypass_obstacle_coordinates((self.x, self.y), (dest_x, dest_y))
            # print("dest: ", dest_x, dest_y)
            # print("location: ", self.x, self.y)
            # print(obstacle.coordinates)
        if self.dest_coords != (0, 0, 0):
            self.drone_control.track_3d(dest_x, dest_y, dest_z, self.recognizable_object)
        self.dest_coords = (dest_x, dest_y, dest_z)

    def track_2d(self, dest_x, dest_y, obstacle=None):
        """
        Moves the drone in the direction of the wanted destination in the xy plain, z destination is the default height.
        If the drone is not active and an obstacle is passed to the function - bypass it.
        :param dest_x: the desired x point.
        :param dest_y: the desired y point.
        :param obstacle: an Obstacle the drone should avoid if its not active.
        """
        self.track_3d(dest_x, dest_y, self.default_height, obstacle)

    def go_home(self, obstacle=None):
        """
        Moves the drone in the direction of its home in the xy plain, z destination is the default height.
        If the drone is not active and an obstacle is passed to the function - bypass it.
        :param obstacle: an Obstacle the drone should avoid if its not active.
        """
        self.track_2d(*self.home, obstacle)

    def track_hitting(self, dest_x, dest_y, dest_z):
        """
        Moves the drone in the direction of the wanted destination in order to hit the balloon.
        :param dest_x: the desired x point.
        :param dest_y: the desired y point.
        :param dest_z: the desired z point
        """
        self.dest_coords = (dest_x, dest_y, dest_z)
        self.drone_control.track_hitting(dest_x, dest_y, dest_z, self.recognizable_object)

    def track_hitting2(self, dest_x, dest_y, dest_z):
        self.dest_coords = (dest_x, dest_y, dest_z)
        self.drone_control.track_hitting2(dest_x, dest_y, dest_z, self.recognizable_object, self.start_hit_vx, self.start_hit_vy)

    def track_descending(self, obstacle=None):
        """
        Moves the drone in the direction of its home in the xy plain while descending.
        If the drone is not active and an obstacle is passed to the function - bypass it.
        :param obstacle: an Obstacle the drone should avoid if its not active.
        """
        dest_x, dest_y = self.home
        if not self.active and obstacle:
            dest_x, dest_y = obstacle.bypass_obstacle_coordinates((self.x, self.y), (dest_x, dest_y))
        self.drone_control.track_descending(dest_x, dest_y, self.recognizable_object)
        self.dest_coords = (dest_x, dest_y, self.default_height)

    def track_descending_2drones(self, other_drone):
        """
        Moves the drone in the away from the other drone in the xy plain while descending.
        :param other_drone: the other drone's Drone object.
        """
        dest_x = 2 * self.x - other_drone.x  # add vector other_drone-self to the location vector of self
        dest_y = 2 * self.y - other_drone.y
        self.drone_control.track_descending(dest_x, dest_y, self.recognizable_object)
        self.dest_coords = (dest_x, dest_y, self.default_height)

    def set_obstacle(self, left_cam):
        """
        Sets the obstacle the drone and its destination creates in the space.
        :param left_cam: the left camera's Camera object.
        """
        if self.dest_coords == (0, 0, 0):
            self.obstacle = None
            return
        self.obstacle = Obstacle(self, left_cam)
