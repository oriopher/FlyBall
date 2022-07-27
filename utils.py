from camera import Camera
import numpy as np
import cv2

def phys_to_left_pix_img(x_cm, y_cm, z_cm, image, cam : Camera): # image is a direct image from the camera and not image3d
    x_n_pix = image.shape[1]
    z_n_pix = image.shape[0]

    return phys_to_left_pix(x_cm, y_cm, z_cm, x_n_pix, z_n_pix, cam.fov_horz)

def phys_to_left_pix(x_cm, y_cm, z_cm, x_n_pix, z_n_pix, cam_fov):
    d_x = x_n_pix / 2 / np.tan(cam_fov / 2)
    x_pix = int(x_n_pix / 2 + d_x * x_cm / y_cm)
    d_z = z_n_pix / 2 / np.tan(cam_fov / 2)
    z_pix = int(z_n_pix / 2 - d_z * z_cm / y_cm)
    return x_pix, z_pix 


def image_with_circle(cam : Camera, show_img, coords_phys, rad_phys, color = (240, 240, 240), thickness = 3):
    if coords_phys == (0, 0, 0):
        return show_img
    x_phys, y_phys, z_phys = coords_phys
    radius = phys_to_left_pix_img(x_phys + rad_phys, y_phys, z_phys, show_img, cam)[0] - phys_to_left_pix_img(x_phys, y_phys, z_phys, show_img, cam)[0]
    coordinates = phys_to_left_pix_img(x_phys, y_phys, z_phys, show_img, cam)
    show_img = cv2.circle(show_img, coordinates, radius, color, thickness=thickness)

    return show_img