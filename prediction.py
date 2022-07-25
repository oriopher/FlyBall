from image_3d import Image3D
import numpy as np
from scipy.integrate import odeint


class BallPredictor:
    def __init__(self, image_3d: Image3D):
        self.time = image_3d.time
        self.x_0 = image_3d.phys_x_balloon / 100
        self.y_0 = image_3d.phys_y_balloon / 100
        self.z_0 = image_3d.phys_z_balloon / 100
        self.v_x_0 = image_3d.velocity_x_balloon / 100
        self.v_y_0 = image_3d.velocity_y_balloon / 100
        self.v_z_0 = image_3d.velocity_z_balloon / 100
        self.theta = np.arctan2(self.v_y_0, self.v_x_0)

        print("x, y, z : %.2f. %.2f, %.2f" % (self.x_0 * 100, self.y_0 * 100, self.z_0 * 100))
        print("vx, vy, vz, theta : %.2f, %.2f, %.2f, %.2f" % (
        self.v_x_0 * 100, self.v_y_0 * 100, self.v_z_0 * 100, self.theta))

    def get_prediction(self, time):
        r = 0.11  # in meters
        g = 9.7803  # Gravitational constatnt
        rho = 1.225  # Air density kg/m^3
        air_mass = 4 / 3 * np.pi * r ** 3 * rho
        m = air_mass + 0.00146  # Balloon mass.
        C_d = 0.47  # Dimensionless drag constant
        A = np.pi * r ** 2  # Balloon cross setion in m^2
        V_t = np.sqrt(2 * m * g / (C_d * A * rho))  # Terminal velocity

        v_z = V_t * (self.v_z_0 - V_t * np.tan(time * g / V_t)) / (V_t + self.v_z_0 * np.tan(time * g / V_t))
        z = self.z_0 + V_t ** 2 / (2 * g) * np.log((self.v_z_0 ** 2 + V_t ** 2) / (v_z ** 2 + V_t ** 2))
        v_xy_0 = np.sqrt(self.v_x_0 ** 2 + self.v_y_0 ** 2)
        # v_xy = V_t**2 * v_xy_0 / (V_t**2 + g * v_xy_0 * time)
        # v_x = v_xy * np.cos(self.theta)
        # v_y =  v_xy * np.sin(self.theta)
        d_xy = V_t ** 2 / g * np.log((V_t ** 2 + g * v_xy_0 * time) / V_t ** 2)
        x = self.x_0 + d_xy * np.cos(self.theta)
        y = self.y_0 + d_xy * np.sin(self.theta)

        x, y, z = x * 100, y * 100, z * 100
        return x, y, z  # cm


class NumericBallPredictor:
    r = 0.1  # in meters
    g = 9.7803  # Gravitational constant
    rho = 1.225  # Air density kg/m^3
    V = 4 / 3 * np.pi * r ** 3  # Balloon Volume
    air_mass = V * rho
    C_d = 0.47  # Dimensionless drag constant
    A = np.pi * r ** 2  # Balloon cross section in m^2
    B = 0.5*rho*A*C_d  # Buoyancy
    rubber_weight = 2.5 * 10 ** -3
    m = air_mass + rubber_weight  # Balloon mass.

    XY_BOUND = 5 / 100
    Z_BOUND = 10 / 100

    def __init__(self, image_3d: Image3D):
        self.time = image_3d.time
        self.x_0 = image_3d.phys_x_balloon / 100
        self.y_0 = image_3d.phys_y_balloon / 100
        self.z_0 = image_3d.phys_z_balloon / 100
        self.v_x_0 = image_3d.velocity_x_balloon / 100
        self.v_y_0 = image_3d.velocity_y_balloon / 100
        self.v_z_0 = image_3d.velocity_z_balloon / 100
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

    def get_optimal_hitting_point(self):
        times = np.concatenate((np.arange(0, 0.51, 0.25), np.arange(0.6, 2.01, 0.1), np.arange(2.25, 3.01, 0.25)))
        preds = self._prepare_predictions(times)
        mask = np.any([preds[:, 0] <=self.XY_BOUND, preds[:, 3] >= self.Z_BOUND], axis=0)
        times = times[mask]
        times = np.linspace(times[0], times[1], 20)
