from camera import Camera
import numpy as np
import cv2

def phys_to_left_pix(x_cm, y_cm, z_cm, image, cam : Camera): # image is a direct image from the camera and not image3d
    x_n_pix = image.shape[1]
    z_n_pix = image.shape[0]
    d_x = x_n_pix / 2 / np.tan(cam.fov / 2)
    x_pix = (int)(d_x * x_cm / y_cm)
    d_z = z_n_pix / 2 / np.tan(cam.fov / 2)
    z_pix = (int)(d_z * z_cm / y_cm)
    return x_pix, z_pix 