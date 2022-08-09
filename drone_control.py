import numpy as np


class DroneControl:

    def connect(self):
        raise NotImplemented

    def takeoff(self):
        raise NotImplemented

    def land(self):
        raise NotImplemented

    def stop(self):
        raise NotImplemented

    def track_descending(self, dest_x, dest_y, recognizable_object):
        raise NotImplemented

    def track_3d(self, dest_x, dest_y, dest_z, recognizable_object):
        raise NotImplemented

    def track_hitting(self, dest_x, dest_y, dest_z, recognizable_object):
        raise NotImplemented

    def bypass_obstacle_coordinates(self, source, target, obstacle):
        raise NotImplemented

    @staticmethod
    def _velocity_control_function(cm_rel, real_velocity, min_vel, max_vel, lower_bound, stopping_vel, a_sqrt, a_linear,
                                   b_param, c_param):
        limit = max(b_param * abs(real_velocity), lower_bound)

        if abs(cm_rel) < limit:
            if abs(cm_rel) < lower_bound:
                velocity = 0
            else:
                velocity = np.sign(real_velocity) * stopping_vel

        else:
            velocity_pot = int(min(max(a_sqrt * np.sqrt(abs(cm_rel) - limit),
                                       a_linear * (abs(cm_rel) - limit) - c_param) + min_vel,
                                   max_vel))
            velocity = -np.sign(cm_rel) * velocity_pot

        return velocity
