import cv2
import numpy as np

from xy_display import XYDisplay


def phys_to_left_pix_img(x_cm, y_cm, z_cm, cam):
    return phys_to_left_pix(x_cm, y_cm, z_cm, cam.last_capture.x_n_pix, cam.last_capture.z_n_pix, cam.fov_horz,
                            cam.fov_vert)


def phys_to_left_pix(x_cm, y_cm, z_cm, x_n_pix, z_n_pix, cam_fov_horz, cam_fov_vert):
    if y_cm == 0:
        return 0, 0
    d_x = x_n_pix / 2 / np.tan(cam_fov_horz / 2)
    x_pix = int(x_n_pix / 2 + d_x * x_cm / y_cm)
    d_z = z_n_pix / 2 / np.tan(cam_fov_vert / 2)
    z_pix = int(z_n_pix / 2 - d_z * z_cm / y_cm)
    return x_pix, z_pix


def image_with_circle(cam, show_img, coords_phys, rad_phys, color=(240, 240, 240), thickness=3):
    if not np.any(coords_phys):
        return show_img
    x_phys, y_phys, z_phys = coords_phys
    radius = phys_to_left_pix_img(x_phys + rad_phys, y_phys, z_phys, cam)[0] - \
             phys_to_left_pix_img(x_phys, y_phys, z_phys, cam)[0]
    coordinates = phys_to_left_pix_img(x_phys, y_phys, z_phys, cam)
    if radius > 0:
        show_img = cv2.circle(show_img, coordinates, radius, color, thickness=thickness)

    return show_img


def image_to_show(show_img, frames, detection_sign=True, texts=None, text_color=(250, 250, 250)):
    for i, frame in enumerate(frames):
        if detection_sign and frame.x != 0 and frame.y != 0:
            show_img = cv2.circle(show_img, (int(frame.x), int(frame.y)), 15, (0, 0, 0), 3)
            if texts and texts[i]:
                show_img = cv2.putText(show_img, texts[i], (int(frame.x), int(frame.y)),
                                       cv2.FONT_HERSHEY_DUPLEX, 1, text_color, 2, cv2.LINE_AA)

    return show_img


def display_frames(balloon, drones, left_cam, right_cam, borders):
    recognizable_objects = [balloon] + [drone.recognizable_object for drone in drones]
    texts_coor = ["c({:.0f},{:.0f},{:.0f})".format(recognizable_object.x, recognizable_object.y, recognizable_object.z)
                  for recognizable_object in recognizable_objects]
    texts_vel = [
        "v({:.0f},{:.0f},{:.0f})".format(recognizable_object.vx, recognizable_object.vy, recognizable_object.vz)
        for recognizable_object in recognizable_objects]

    left_img = image_to_show(left_cam.last_capture.image,
                             [recognizable_object.frame_left for recognizable_object in recognizable_objects], True)

    for drone in drones:
        if np.any(drone.dest_coords):
            left_img = image_with_circle(left_cam, left_img, drone.dest_coords, rad_phys=7, thickness=2)

    right_img = image_to_show(right_cam.last_capture.image,
                              [recognizable_object.frame_right for recognizable_object in recognizable_objects], True,
                              texts_coor, (240, 150, 240))

    obstacle = None
    for drone in drones:
        if drone.start and drone.active:
            obstacle = drone.obstacle
            break

    xy_display = XYDisplay.get_xy_display(borders, balloon, drones, obstacle)
    return left_img, right_img, xy_display
