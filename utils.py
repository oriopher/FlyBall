from camera import Camera
import numpy as np

def phys_to_left_pix(x_cm, y_cm, z_cm, x_n_pix, z_n_pix, cam : Camera): # n_pix is number of pixels in x axis.
    d_x = x_n_pix / 2 / np.tan(cam.fov / 2)
    x_pix = d_x * x_cm / y_cm
    d_z = z_n_pix / 2 / np.tan(cam.fov / 2)
    z_pix = d_z * z_cm / y_cm 
    return x_pix, z_pix