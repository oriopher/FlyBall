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
    B = 1
    M = 1
    RHO = 1
    V = 1
    g = 9.8

    def __init__(self, image_3d: Image3D, latest_time: float, num_predictions: int):
        self.time = image_3d.time
        self.x_0 = image_3d.phys_x_balloon
        self.y_0 = image_3d.phys_y_balloon
        self.z_0 = image_3d.phys_z_balloon
        self.v_x_0 = image_3d.velocity_x_balloon
        self.v_y_0 = image_3d.velocity_y_balloon
        self.v_z_0 = image_3d.velocity_z_balloon
        self.theta = np.arctan2(self.v_y_0, self.v_x_0)
        self.v_xy_0 = np.sqrt(self.v_x_0 ** 2 + self.v_y_0 ** 2)
        self.times = np.linspace(0, latest_time, num_predictions)
        self.xs, self.ys, self.zs = self.prepare_predictions(self.times)
        self.predictions = np.array([self.times, self.xs, self.ys, self.zs]).T

    @staticmethod
    def derivative_func(variables, time, buoyancy, mass, density, volume, gravity, theta):
        c1 = -(buoyancy / mass) * (variables[0] ** 2 + variables[1] ** 2)
        c2 = (density * volume / mass - 1) * gravity
        return np.array([c1 * np.sin(theta), c2 + c1 * np.cos(), variables[0], variables[1]])

    def prepare_predictions(self, times):
        sol = odeint(NumericBallPredictor.derivative_func, np.array([self.v_xy_0, self.v_z_0, 0, self.z_0]), times,
                     args=(self.B, self.M, self.RHO, self.V, self.g, self.theta))
        d_xy = sol[:, 2]
        z = sol[:, 3]
        x = self.x_0 + d_xy * np.cos(self.theta)
        y = self.y_0 + d_xy * np.sin(self.theta)

        x, y, z = x * 100, y * 100, z * 100
        return x, y, z
