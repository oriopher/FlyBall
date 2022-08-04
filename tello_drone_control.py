from drone_control import DroneControl
import numpy as np
from tello import Tello
from scipy.spatial import distance_matrix
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra


class TelloDroneControl(DroneControl):

    def __init__(self, iface_ip):
        self.iface_ip = iface_ip
        self.tello = None

    def get_battery(self):
        return self.tello.get_battery()

    def connect(self):
        if not self.tello:
            self.tello = Tello(iface_ip=self.iface_ip)
        self.tello.connect()

    def takeoff(self):
        self.tello.takeoff()

    def land(self):
        self.tello.land()

    def track_3d(self, dest_x: float, dest_y: float, dest_z: float, recognizable_object):
        x_cm_rel = dest_x - recognizable_object.x
        y_cm_rel = dest_y - recognizable_object.y
        z_cm_rel = dest_z - recognizable_object.z
        left_right = self.velocity_control_function(x_cm_rel, recognizable_object.vx, 'x')
        for_back = self.velocity_control_function(y_cm_rel, recognizable_object.vy, 'y')
        up_down = self.velocity_control_function(z_cm_rel, recognizable_object.vz, 'z')
        self.tello.send_rc_control(left_right, for_back, up_down, 0)

    def track_hitting(self, dest_x, dest_y, dest_z, recognizable_object):
        rx = dest_x - recognizable_object.x
        ry = dest_y - recognizable_object.y
        rz = dest_z - recognizable_object.z
        r = np.sqrt(rx ** 2 + ry ** 2 + rz ** 2)
        theta = np.arccos(rz / r)
        phi = np.arctan2(ry, rx)
        MAX_VEL = 100
        MIN_VEL = 9
        A = 1.7
        vz = MAX_VEL
        vx = min(int(A * (vz - MIN_VEL) * np.tan(theta) * np.cos(phi)) + MIN_VEL, MAX_VEL)
        vy = min(int(A * (vz - MIN_VEL) * np.tan(theta) * np.sin(phi)) + MIN_VEL, MAX_VEL)

        left_right, for_back, up_down = -vx, -vy, vz
        self.tello.send_rc_control(left_right, for_back, up_down, 0)

    def track_descending(self, dest_x, dest_y, recognizable_object):
        x_cm_rel = dest_x - recognizable_object.x
        y_cm_rel = dest_y - recognizable_object.y
        left_right = self.velocity_control_function(x_cm_rel, recognizable_object.vx, 'x')
        for_back = self.velocity_control_function(y_cm_rel, recognizable_object.vy, 'y')

        self.tello.send_rc_control(left_right, for_back, -100, 0)

    def stop(self):
        self.tello.send_rc_control(0, 0, 0, 0)

    def velocity_control_function(self, cm_rel, real_velocity, direction):
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

        velocity = int(self._velocity_control_function(cm_rel, real_velocity, MIN_VEL, MAX_VEL,
                                                       LOWER_BOUND, STOPPING_VEL, A_SQRT, A_LINEAR, B, C))

        if direction == 'x':
            return velocity
        elif direction == 'y':
            return velocity
        elif direction == 'z':
            return -velocity
