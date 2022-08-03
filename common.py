import numpy as np
import cv2
import os
from consts import *

from xy_display import draw_xy_display


def phys_to_left_pix_img(x_cm, y_cm, z_cm, image, cam):  # image is a direct image from the camera and not image3d
    x_n_pix = image.shape[1]
    z_n_pix = image.shape[0]

    return phys_to_left_pix(x_cm, y_cm, z_cm, x_n_pix, z_n_pix, cam.fov_horz, cam.fov_vert)


def phys_to_left_pix(x_cm, y_cm, z_cm, x_n_pix, z_n_pix, cam_fov_horz, cam_fov_vert):
    d_x = x_n_pix / 2 / np.tan(cam_fov_horz / 2)
    x_pix = int(x_n_pix / 2 + d_x * x_cm / y_cm)
    d_z = z_n_pix / 2 / np.tan(cam_fov_vert / 2)
    z_pix = int(z_n_pix / 2 - d_z * z_cm / y_cm)
    return x_pix, z_pix


def image_with_circle(cam, show_img, coords_phys, rad_phys, color=(240, 240, 240), thickness=3):
    if not np.any(coords_phys):
        return show_img
    x_phys, y_phys, z_phys = coords_phys
    radius = phys_to_left_pix_img(x_phys + rad_phys, y_phys, z_phys, show_img, cam)[0] - phys_to_left_pix_img(x_phys, y_phys, z_phys, show_img, cam)[0]
    coordinates = phys_to_left_pix_img(x_phys, y_phys, z_phys, show_img, cam)
    if radius > 0:
        show_img = cv2.circle(show_img, coordinates, radius, color, thickness=thickness)

    return show_img


def reachability(distance, offset=0.6):
    # distance in cm, only one axis
    plot = np.array([[0, 0.95],
                     [2, 2],
                     [30, 2.76],
                     [50, 2.76],
                     [70, 2.93],
                     [90, 4.15]])

    for i in range(len(plot) - 1):
        if plot[i, 0] <= distance <= plot[i + 1, 0]:
            a = (plot[i + 1, 1] - plot[i, 1]) / (plot[i + 1, 0] - plot[i, 0])
            b = - a * plot[i, 0] + plot[i, 1]
            return a * distance + b + offset

    return plot[-1, 1]


def write_colors(filename, recognizable_objects):
    file_text = "".join([recognizable_object.colors_string for recognizable_object in recognizable_objects])
    if os.path.exists(filename):
        os.remove(filename)
    with open(filename, 'w') as f:
        f.write(file_text)
        print("Colors Saved")


def read_colors(filename, recognizable_objects):
    if not os.path.exists(filename):
        print("ERROR: text_colors file does not exist")
        return

    with open(filename, 'r') as f:
        lines = f.readlines()

    num_of_bounds = 4
    for i, recognizable_object in enumerate(recognizable_objects):
        bounds = [lines[i * num_of_bounds + j] for j in range(num_of_bounds)]
        recognizable_object.save_colors(bounds)
    print("Colors Loaded")


def image_to_show(show_img, frames, detection_sign=True, texts=None, text_color=(250, 250, 250)):
    for i, frame in enumerate(frames):
        if detection_sign and frame.x != 0 and frame.y != 0:
            show_img = cv2.circle(show_img, (int(frame.x), int(frame.y)), 15, (0, 0, 0), 3)
            if texts and texts[i]:
                show_img = cv2.putText(show_img, texts[i], (int(frame.x), int(frame.y)),
                                       cv2.FONT_HERSHEY_DUPLEX, 1, text_color, 2, cv2.LINE_AA)

    return show_img


def display_frames(balloon, drone, left_cam, right_cam, borders):
    recognizable_objects = [balloon, drone.recognizable_object]
    texts_coor = ["c({:.0f},{:.0f},{:.0f})".format(recognizable_object.x, recognizable_object.y, recognizable_object.z)
                  for recognizable_object in recognizable_objects]
    texts_vel = [
        "v({:.0f},{:.0f},{:.0f})".format(recognizable_object.vx, recognizable_object.vy, recognizable_object.vz)
        for recognizable_object in recognizable_objects]

    left_img = image_to_show(left_cam.last_capture.image,
                             [recognizable_object.frame_left for recognizable_object in recognizable_objects],
                             True, texts_coor, (150, 250, 200))
    left_img = borders.draw_borders(left_img, balloon, color_in=(0, 240, 0), color_out=(0, 0, 240))
    if drone.dest_coords != (0, 0, 0):
        left_img = image_with_circle(left_cam, left_img, drone.dest_coords, rad_phys=7, thickness=2)
    cv2.imshow("left_cam", left_img)
    right_img = image_to_show(right_cam.last_capture.image,
                              [recognizable_object.frame_right for recognizable_object in recognizable_objects], True,
                              texts_vel, (240, 150, 240))
    cv2.imshow("right_cam", right_img)
    draw_xy_display(borders, recognizable_objects, drone.dest_coords[0], drone.dest_coords[1])


def calc_linear_eq(coor1, coor2):
    m = (coor2[1] - coor1[1]) / (coor2[0] - coor1[0])
    b = coor2[1] - m * coor2[0]
    return m, b
