from drone_control import DroneControl
import numpy as np
from tello import Tello


class TelloDroneControl(DroneControl):
    """
    A class for controlling a Tello drone.
    """

    def __init__(self, iface_ip):
        """
        Initialize the tello drone controller.
        :param iface_ip: the ip of the interface that connects to the specific drone.
        """
        self.iface_ip = iface_ip
        self.tello = None

    def connect(self):
        """
        Connects to the tello drone.
        If the Tello object is not set yet - than set it.
        """
        if not self.tello:
            self.tello = Tello(iface_ip=self.iface_ip)
        self.tello.connect()

    def takeoff(self):
        """
        Commands the tello drone to takeoff.
        """
        self.tello.takeoff()

    def land(self):
        """
        Commands the tello drone to land.
        """
        self.tello.land()

    def stop(self):
        """
        Commands the tello drone to stop.
        """
        self.tello.send_rc_control(0, 0, 0, 0)

    def get_battery(self):
        """
        Queries the battery of the tello drone.
        :return: the battery percentage of the tello drone.
        """
        return self.tello.get_battery()

    def track_3d(self, dest_x: float, dest_y: float, dest_z: float, recognizable_object):
        """
        Moves the tello drone in the direction of the inputted destination.
        :param dest_x: the desired x point.
        :param dest_y: the desired y point.
        :param dest_z: the desired z point
        :param recognizable_object: the drones RecognizableObject.
        """
        x_cm_rel = dest_x - recognizable_object.x
        y_cm_rel = dest_y - recognizable_object.y
        z_cm_rel = dest_z - recognizable_object.z
        left_right = self.velocity_control_function(x_cm_rel, recognizable_object.vx, 'x')
        for_back = self.velocity_control_function(y_cm_rel, recognizable_object.vy, 'y')
        up_down = self.velocity_control_function(z_cm_rel, recognizable_object.vz, 'z')
        self.tello.send_rc_control(left_right, for_back, up_down, 0)

    def track_hitting(self, dest_x, dest_y, dest_z, recognizable_object):
        """
        Moves the tello drone in the direction of the inputted destination in order to hit the balloon.
        :param dest_x: the desired x point.
        :param dest_y: the desired y point.
        :param dest_z: the desired z point
        :param recognizable_object: the drones RecognizableObject.
        """
        rx = dest_x - recognizable_object.x
        ry = dest_y - recognizable_object.y
        rz = dest_z - recognizable_object.z
        r = np.sqrt(rx ** 2 + ry ** 2 + rz ** 2)
        theta = np.arccos(rz / r)
        phi = np.arctan2(ry, rx)
        MAX_VEL = 100
        MIN_VEL = 9
        A = 1.7
        B = 1
        vz = MAX_VEL
        vx = min(int(A * (vz - MIN_VEL) * np.tan(theta) * np.cos(phi) - B * recognizable_object.vx) + MIN_VEL, MAX_VEL)
        vy = min(int(A * (vz - MIN_VEL) * np.tan(theta) * np.sin(phi) - B * recognizable_object.vy) + MIN_VEL, MAX_VEL)

        left_right, for_back, up_down = -vx, -vy, vz
        self.tello.send_rc_control(left_right, for_back, up_down, 0)

    def track_hitting2(self, dest_x, dest_y, dest_z, recognizable_object, vx0, vy0):
        rx = dest_x - recognizable_object.x
        ry = dest_y - recognizable_object.y
        rz = dest_z - recognizable_object.z
        r = np.sqrt(rx ** 2 + ry ** 2 + rz ** 2)
        theta = np.arccos(rz / r)
        phi = np.arctan2(ry, rx)
        MAX_VEL = 100
        MIN_VEL = 9
        A = 1.7
        B = 2
        vz = MAX_VEL
        vx = min(int(A * (vz - MIN_VEL) * np.tan(theta) * np.cos(phi) - B * vx0) + MIN_VEL, MAX_VEL)
        vy = min(int(A * (vz - MIN_VEL) * np.tan(theta) * np.sin(phi) - B * vy0) + MIN_VEL, MAX_VEL)

        left_right, for_back, up_down = -vx, -vy, vz
        self.tello.send_rc_control(left_right, for_back, up_down, 0)

    def track_descending(self, dest_x, dest_y, recognizable_object):
        """
        Moves the tello drone in the direction of the inputted destination while descending.
        :param dest_x: the desired x point.
        :param dest_y: the desired y point.
        :param recognizable_object: the drones RecognizableObject.
        """
        x_cm_rel = dest_x - recognizable_object.x
        y_cm_rel = dest_y - recognizable_object.y
        left_right = self.velocity_control_function(x_cm_rel, recognizable_object.vx, 'x')
        for_back = self.velocity_control_function(y_cm_rel, recognizable_object.vy, 'y')

        self.tello.send_rc_control(left_right, for_back, -100, 0)

    def velocity_control_function(self, cm_rel, real_velocity, direction):
        """
        Calculates the velocity to give the tello drone in a certain axis.
        :param cm_rel: the relative distance from the target in the axis.
        :param real_velocity: the velocity of the drone in the axis.
        :param direction: the axis in which to calculate the velocity.
        :return: the velocity to give the drone in the inputted axis.
        """
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
