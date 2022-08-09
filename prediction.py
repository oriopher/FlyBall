import numpy as np
from scipy.integrate import odeint

from recognizable_object import RecognizableObject


class NumericBallPredictor:
    # r = (0.65+0.71)/4/np.pi  # in meters
    r = (0.695+0.75)/4/np.pi  # in meters
    g = 9.807  # Gravitational constant
    rho = 1.183  # Air density kg/m^3
    V = 4 / 3 * np.pi * r ** 3  # Balloon Volume
    disp_air_mass = V * rho
    C_d = 0.78  # Dimensionless drag constant
    A = np.pi * r ** 2  # Balloon cross section in m^2
    B = 0.5*rho*A*C_d  # Buoyancy
    # balloon_weight = 1.5 * 10 ** -3  # kg
    balloon_weight = 1.75 * 10 ** -3  # kg
    m = disp_air_mass + balloon_weight  # Balloon mass.

    def __init__(self, balloon: RecognizableObject):
        self.time = balloon.time
        self.x_0 = balloon.x / 100
        self.y_0 = balloon.y / 100
        self.z_0 = balloon.z / 100
        self.v_x_0 = balloon.vx / 100
        self.v_y_0 = balloon.vy / 100
        self.v_z_0 = balloon.vz / 100
        self.phi = np.arctan2(self.v_y_0, self.v_x_0)
        self.v_xy_0 = np.sqrt(self.v_x_0 ** 2 + self.v_y_0 ** 2)
        self.theta = np.arctan2(self.v_xy_0, self.v_z_0)

        
    @staticmethod
    def _derivative_func(variables, time, b, mass, density, volume, gravity):
        theta = np.arctan2(variables[0], variables[1])
        c1 = -(b / mass) * (variables[0] ** 2 + variables[1] ** 2)
        c2 = (density * volume / mass - 1) * gravity
        return np.array([c1 * np.sin(theta), c2 + c1 * np.cos(theta), variables[0], variables[1]])

    def _prepare_predictions(self, times):
        return odeint(NumericBallPredictor._derivative_func, np.array([self.v_xy_0, self.v_z_0, 0, self.z_0]), times,
                     args=(self.B, self.m, self.rho, self.V, self.g))

    def _solution_to_coords(self, sol):
        d_xy = sol[:, 2]
        z = sol[:, 3]
        x = self.x_0 + d_xy * np.cos(self.phi)
        y = self.y_0 + d_xy * np.sin(self.phi)

        x, y, z = x * 100, y * 100, z * 100
        return np.array([x, y, z])

    def get_prediction(self, time):
        sol = self._prepare_predictions(np.linspace(0, time, 2))
        return self._solution_to_coords(sol)[:, 1]

    def _get_optimal_hitting_point_for_times(self, times, xy_vel_bounds, z_bound):
        preds = self._prepare_predictions(np.insert(times, 0, 0))[1:]
        mask = np.all([preds[:, 3] >= z_bound, np.abs(preds[:, 0]) <= xy_vel_bounds], axis=0)
        return times[mask], preds[mask]

    def get_optimal_hitting_point(self, start_time=0, end_time=3, jump=0.1, xy_vel_bound=5/100, z_bound=30/100):
        # final precision of the time of the optimal hit is jump/10
        times = np.arange(start_time, end_time + jump / 2, jump)
        times, preds = self._get_optimal_hitting_point_for_times(times, xy_vel_bound, z_bound)
        if len(preds) == 0:
            return 0, (0, 0, 0)
        if times[0] == start_time:
            return start_time, self._solution_to_coords(preds)[:, 0]
        time = times[0]
        new_jump = jump / 10
        times = np.arange(time - jump, time + new_jump / 2, new_jump)
        times, preds = self._get_optimal_hitting_point_for_times(times, xy_vel_bound, z_bound)
        return times[0], self._solution_to_coords(preds)[:, 0]

    def get_prediction_height(self, height, vel_limit=30):
        seconds = 4
        fps = 50
        times = np.linspace(0,seconds,seconds*fps)

        return self.get_prediction_height_rec(times, 0, seconds*fps-1, height, 1/fps, vel_limit)

    def get_prediction_height_rec(self, times, left, right, height, jump=0.03, vel_limit=30):
        if left >= right - 1:
            return left, self.get_prediction(times[left])

        middle = int(left/2 + right/2)
        x1, y1, z1 = self.get_prediction(times[middle])
        x2, y2, z2 = self.get_prediction(times[middle+1])

        if z2 > z1:
            return self.get_prediction_height_rec(times, middle, right, height, jump, vel_limit)

        if abs(x2-x1)/jump < vel_limit and abs(y2-y1)/jump < vel_limit:
            return middle, (x1, y1, z1)

        if z2 >= height:
            return self.get_prediction_height_rec(times, middle, right, height, jump, vel_limit)
        if z1 <= height:
            return self.get_prediction_height_rec(times, left, middle, height, jump, vel_limit)

