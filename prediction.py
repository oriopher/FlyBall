from image_3d import Image3D
import numpy as np


class BallPredictor:

    def __init__(self, image_3d : Image3D):
        self.time = image_3d.time
        self.x_0 = image_3d.phys_x_balloon
        self.y_0 = image_3d.phys_y_balloon
        self.z_0 = image_3d.phys_z_balloon
        self.v_x_0 = image_3d.velocity_x_balloon
        self.v_y_0 = image_3d.velocity_y_balloon
        self.v_z_0 = image_3d.velocity_z_balloon
        self.theta = np.arctan(self.v_y_0 / self.v_x_0)

    def get_prediction(self, time):
        g = 9.7803 # Gravitational constatnt
        m = 0.002 # Balloon mass.
        C_d = 0.47 # Dimensionless drag constant
        A = 0.1809557  # Balloon cross setion in m^2
        rho = 1.225 # Air density kg/m^3
        V_t = np.sqrt(2 * m * g / (C_d * A * rho)) # Terminal velocity

        v_z = V_t * (self.v_z_0 - V_t * np.tan(time * g / V_t)) / (self.v_z_0 + V_t * np.tan(time * g / V_t))
        z = self.z_0 + V_t**2 / (2 * g) * np.log( (self.v_z_0**2 + V_t**2) / (v_z**2 + V_t**2) )
        v_xy_0  = np.sqrt( self.v_x_0**2 + self.v_y_0**2 )
        v_xy = V_t**2 * v_xy_0 / (V_t**2 + g * v_xy_0 * time)
        v_x = v_xy * np.cos(self.theta)
        v_y =  v_xy * np.sin(self.theta)
        d_xy  = V_t**2 / g * np.log( (V_t**2 + g * v_xy_0 * time) / V_t**2 )
        x = self.x_0 + d_xy * np.cos(self.theta)
        y = self.y_0 + d_xy * np.sin(self.theta)

        return x, y, z # cm