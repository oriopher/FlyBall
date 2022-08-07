import numpy as np
from scipy.integrate import odeint

from recognizable_object import RecognizableObject


class NumericBallPredictor:
    """
    A predictor of the balloon's location in future time depending on its location and velocity
    that solves the balloon's equations of motion using a numeric ODE solver.
    """
    r = 0.113  # in meters
    g = 9.807  # Gravitational constant
    rho = 1.183  # Air density kg/m^3
    V = 4 / 3 * np.pi * r ** 3  # Balloon Volume
    disp_air_mass = V * rho
    C_d = 0.78  # Dimensionless drag constant
    A = np.pi * r ** 2  # Balloon cross section in m^2
    B = 0.5*rho*A*C_d # A parameter of drag force
    balloon_weight = 1.7 * 10 ** -3
    m = disp_air_mass + balloon_weight  # Balloon mass.

    def __init__(self, balloon: RecognizableObject):
        """
        Initializes the initial location and coordinates of the balloon for the predictor.
        :param balloon: the object representing the balloon.
        """
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
        """
        Calculates the derivative of variables of the ODE at a certain step.
        See the calculations document for details.
        :param variables: the values of the ODE's variables at the begging of the step.
        :param time: the time of the step.
        :param b: a parameter of the drag force on the balloon (B in calculations document)
        :param mass: the mass of the balloon
        :param density: the density of air
        :param volume: the volume of the balloon
        :param gravity: the gravity coefficient
        :return: the derivatives of the variables at this step, according to the ODE equations.
        """
        theta = np.arctan2(variables[0], variables[1])
        c1 = -(b / mass) * (variables[0] ** 2 + variables[1] ** 2)
        c2 = (density * volume / mass - 1) * gravity
        return np.array([c1 * np.sin(theta), c2 + c1 * np.cos(theta), variables[0], variables[1]])

    def _prepare_predictions(self, times):
        """
        :param times: a list of the times (starting from the initial time of creating the instance)
                      in which to return the values of the variables of the ODE.
        :return: a list of the values of the variables of the ODE at the inputted wanted times.
        """
        return odeint(NumericBallPredictor._derivative_func, np.array([self.v_xy_0, self.v_z_0, 0, self.z_0]), times,
                     args=(self.B, self.m, self.rho, self.V, self.g))

    def _solution_to_coords(self, sol):
        """
        Transform a solution of the ODE back to the original coordinates in centimeters.
        :param sol: A solution of the ODE.
        :return: the (x, y, z) coordinates corresponding to the inputted solution.
        """
        d_xy = sol[:, 2]
        z = sol[:, 3]
        x = self.x_0 + d_xy * np.cos(self.phi)
        y = self.y_0 + d_xy * np.sin(self.phi)

        x, y, z = x * 100, y * 100, z * 100
        return np.array([x, y, z])

    def get_prediction(self, time):
        """
        Predict the balloons location at a certain time.
        :param time: a time (0 is the initial time when creating the instance of the predictor).
        :return: a prediction of the (x, y, z) coordinates of the balloon at the inputted time.
        """
        sol = self._prepare_predictions(np.linspace(0, time, 2))
        return self._solution_to_coords(sol)[:, 1]

    def get_optimal_hitting_point(self, start_time=0, end_time=3, time_precision=0.01, xy_vel_bound=5/100, z_bound=30/100):
        """
        Predict an optimal hitting point, which is the first point upholding the following criteria:
        * the balloon's velocity in the XY plains is bellow an inputted bound
        * the height of the balloon is higher than an inputted bound
        The search for the optimal point is only between the inputted start and end times.
        :param start_time: the start time of the search window for the optimal hitting point.
        :param end_time: the end time of the search window for the optimal hitting point.
        :param time_precision: the precision of the time of the optimal hitting point.
        :param xy_vel_bound: the bound on the XY plain velocity, as described above.
        :param z_bound: the bound on the height, as described above.
        :return: the time and location of the prediction of the optimal hitting point in the following format: (t, (x, ,y, z)),
                 if there is no time in which the criteria is upheld at the inputted time bounds (0, (0, 0, 0)) is returned.
        """
        # final precision of the time of the optimal hit is jump/10
        jump = 10 * time_precision
        times = np.arange(start_time, end_time + jump / 2, jump)
        times, preds = self._get_legal_hitting_points_for_times(times, xy_vel_bound, z_bound)
        if len(preds) == 0:
            return 0, (0, 0, 0)
        if times[0] == start_time:
            return start_time, self._solution_to_coords(preds)[:, 0]
        time = times[0]
        # make another cycle of searching for optimal hitting point with the precision magnified by 10
        # (only at the interval that is before the previous cycles optimal hitting point)
        new_jump = jump / 10
        times = np.arange(time - jump, time + new_jump / 2, new_jump)
        times, preds = self._get_legal_hitting_points_for_times(times, xy_vel_bound, z_bound)
        return times[0], self._solution_to_coords(preds)[:, 0]

    def _get_legal_hitting_points_for_times(self, times, xy_vel_bounds, z_bound):
        """
        Get the legal hitting point as describes in 'get_optimal_hitting_point' for certain discrete times.
        :param times: the times to consider for the optimal hitting point.
        :param xy_vel_bounds: the bound on the XY plain velocity.
        :param z_bound: the bound on the height.
        :return: the times and the ODE's solutions of all the points upholding the criteria described in 'get_optimal_hitting_point'.
        """
        preds = self._prepare_predictions(np.insert(times, 0, 0))[1:]
        mask = np.all([preds[:, 3] >= z_bound, np.abs(preds[:, 0]) <= xy_vel_bounds], axis=0)
        return times[mask], preds[mask]

    # def get_prediction_height(self, height, vel_limit=30):
    #     seconds = 4
    #     fps = 50
    #     times = np.linspace(0,seconds,seconds*fps)
    #
    #     return self.get_prediction_height_rec(times, 0, seconds*fps-1, height, 1/fps, vel_limit)
    #
    # def get_prediction_height_rec(self, times, left, right, height, jump=0.03, vel_limit=30):
    #     if left >= right - 1:
    #         return left, self.get_prediction(times[left])
    #
    #     middle = int(left/2 + right/2)
    #     x1, y1, z1 = self.get_prediction(times[middle])
    #     x2, y2, z2 = self.get_prediction(times[middle+1])
    #
    #     if z2 > z1:
    #         return self.get_prediction_height_rec(times, middle, right, height, jump, vel_limit)
    #
    #     if abs(x2-x1)/jump < vel_limit and abs(y2-y1)/jump < vel_limit:
    #         return middle, (x1, y1, z1)
    #
    #     if z2 >= height:
    #         return self.get_prediction_height_rec(times, middle, right, height, jump, vel_limit)
    #     if z1 <= height:
    #         return self.get_prediction_height_rec(times, left, middle, height, jump, vel_limit)

