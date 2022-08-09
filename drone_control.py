import numpy as np


class DroneControl:
    """
    A base class for controlling a drone.
    """

    def connect(self):
        """
        Initializes the connection to the drone.
        """
        raise NotImplemented

    def takeoff(self):
        """
        Commands the drone to takeoff.
        """
        raise NotImplemented

    def land(self):
        """
        Commands the drone to land.
        """
        raise NotImplemented

    def stop(self):
        """
        Commands the drone to stop.
        """
        raise NotImplemented

    def track_descending(self, dest_x, dest_y, recognizable_object):
        """
        Uses the RecognizableObject of the drone to track the inputted destination while descending.
        :param dest_x: the desired x coordinate.
        :param dest_y: the desired y coordinate.
        :param recognizable_object: the drones RecognizableObject.
        """
        raise NotImplemented

    def track_3d(self, dest_x, dest_y, dest_z, recognizable_object):
        """
        Uses the RecognizableObject of the drone to track the inputted destination.
        :param dest_x: the desired x coordinate.
        :param dest_y: the desired y coordinate.
        :param dest_z: the desired z coordinate
        :param recognizable_object: the drones RecognizableObject.
        """
        raise NotImplemented

    def track_hitting(self, dest_x, dest_y, dest_z, recognizable_object):
        """
        Uses the RecognizableObject of the drone to track the inputted destination in order to hit the balloon.
        :param dest_x: the desired x coordinate.
        :param dest_y: the desired y coordinate.
        :param dest_z: the desired z coordinate
        :param recognizable_object: the drones RecognizableObject.
        """
        raise NotImplemented

    @staticmethod
    def _velocity_control_function(cm_rel, real_velocity, min_vel, max_vel, lower_bound, stopping_vel, limit_param,
                                   a_sqrt, a_linear, b_linear):
        """
        Calculates the desired velocity of the drone in one axis in order to reach the destination.
        :param cm_rel: the distance of the drone from the destination in the axis.
        :param real_velocity: the drones velocity in the axis.
        :param min_vel: the minimal velocity to use.
        :param max_vel: the maximal velocity to use.
        :param lower_bound: the bound indicating the drone is close to the destination.
        :param stopping_vel: the stopping velocity to give the drone.
        :param limit_param: a parameter for calculating the limit in which to start stopping.
        :param a_sqrt: a parameter for the root square potential function.
        :param a_linear: the multiply parameter for the linear potential function.
        :param b_linear: the free parameter for the linear potential function.
        :return: the velocity in which the drone should fly in the axis.
        """
        limit = max(limit_param * abs(real_velocity), lower_bound)

        if abs(cm_rel) < limit:
            if abs(cm_rel) < lower_bound:
                velocity = 0
            else:
                velocity = np.sign(real_velocity) * stopping_vel

        else:
            velocity_pot = int(min(max(a_sqrt * np.sqrt(abs(cm_rel) - limit),
                                       a_linear * (abs(cm_rel) - limit) - b_linear) + min_vel,
                                   max_vel))
            velocity = -np.sign(cm_rel) * velocity_pot

        return velocity
