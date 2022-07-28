from camera import Camera
import numpy as np
import cv2

FLOOR_HEIGHT = -100
DRONE_DEFAULT_HEIGHT = FLOOR_HEIGHT + 40

ORI_WEB = Camera(51.3, 0, False)
ORI_PHONE = Camera(66.9, 3, False)
NIR_PHONE = Camera(65, 0, False)
MAYA_WEB = Camera(61, 0, True)
EFRAT_WEB = Camera(61, 2, False)
EFRAT_PHONE = Camera(64, 3, False)
MAYA_PHONE_NIR = Camera(67, 67, 2, False)

NIR_PHONE_NIR = Camera(67, 52, 0, False)
EFRAT_PHONE_NIR = Camera(77, 2, False)

COLORS_FILENAME = "color_bounds.txt"
BORDERS_FILENAME = "borders.txt"


def phys_to_left_pix_img(x_cm, y_cm, z_cm, image, cam : Camera): # image is a direct image from the camera and not image3d
    x_n_pix = image.shape[1]
    z_n_pix = image.shape[0]

    return phys_to_left_pix(x_cm, y_cm, z_cm, x_n_pix, z_n_pix, cam.fov_horz, cam.fov_vert)

def phys_to_left_pix(x_cm, y_cm, z_cm, x_n_pix, z_n_pix, cam_fov_horz, cam_fov_vert):
    d_x = x_n_pix / 2 / np.tan(cam_fov_horz / 2)
    x_pix = int(x_n_pix / 2 + d_x * x_cm / y_cm)
    d_z = z_n_pix / 2 / np.tan(cam_fov_vert / 2)
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


def reachability(distance, offset = 0.2):
    # distance in cm, only one axis
    plot = np.array([[0, 0.95], 
                    [10, 1.35],
                    [30,2.76],
                    [50,2.76],
                    [70,2.93],
                    [90,4.15]])

    for i in range(len(plot)-1): 
        if plot[i,0] <= distance <= plot[i+1,0]:
            a = (plot[i+1,1]-plot[i,1])/(plot[i+1,0] - plot[i,0])
            b = - a * plot[i,0] + plot[i,1]
            return a*distance + b + offset

    return plot[-1,1]