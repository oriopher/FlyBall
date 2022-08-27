import cv2
import numpy as np

from xy_display import XYDisplay


def phys_to_left_pix_img(x_cm, y_cm, z_cm, cam):
    """
    Calculates the pixel location of an inputted point on a frame of an inputted camera (left camera).
    :param x_cm: the x coordinate of the point.
    :param y_cm: the y coordinate of the point.
    :param z_cm: the z coordinate of the point.
    :param cam: the Camera of the the camera in which to calculate the pixel location.
    :return: the pixel location of the point on the frame of the camera.
    """
    return phys_to_left_pix(x_cm, y_cm, z_cm, cam.last_capture.x_n_pix, cam.last_capture.z_n_pix, cam.fov_horz,
                            cam.fov_vert)


def phys_to_left_pix(x_cm, y_cm, z_cm, x_n_pix, z_n_pix, cam_fov_horz, cam_fov_vert):
    """

    Calculates the pixel location of an inputted point on a frame.
    :param x_cm: the x coordinate of the point.
    :param y_cm: the y coordinate of the point.
    :param z_cm: the z coordinate of the point.
    :param x_n_pix: the number of pixels in the x axis on the frame.
    :param z_n_pix: the number of pixels in the z axis on the frame.
    :param cam_fov_horz: the camera's field of view horizontal angle.
    :param cam_fov_vert: the camera's field of view vertical angle.
    :return: the pixel location of the point on the frame.
    """
    if y_cm == 0:
        return 0, 0
    d_x = x_n_pix / 2 / np.tan(cam_fov_horz / 2)
    x_pix = int(x_n_pix / 2 + d_x * x_cm / y_cm)
    d_z = z_n_pix / 2 / np.tan(cam_fov_vert / 2)
    z_pix = int(z_n_pix / 2 - d_z * z_cm / y_cm)
    return x_pix, z_pix


def image_with_circle(cam, show_img, coords_phys, rad_phys, color=(240, 240, 240), thickness=3):
    """
    Adds a circle to an image.
    :param cam: the camera from which the frame of the image is.
    :param show_img: the image to add the the circle on.
    :param coords_phys: the physical coordinates of the center of the circle to add to the image.
    :param rad_phys: the radius of the circle.
    :param color: the color of the circle.
    :param thickness: the thickness of the circle.
    :return: the image with the circle.
    """
    if not np.any(coords_phys):
        return show_img
    x_phys, y_phys, z_phys = coords_phys
    radius = phys_to_left_pix_img(x_phys + rad_phys, y_phys, z_phys, cam)[0] - \
             phys_to_left_pix_img(x_phys, y_phys, z_phys, cam)[0]
    coordinates = phys_to_left_pix_img(x_phys, y_phys, z_phys, cam)
    if radius > 0:
        show_img = cv2.circle(show_img, coordinates, radius, color, thickness=thickness)

    return show_img


def image_to_show(show_img, objects_in_frames, detection_sign=True, texts=None, text_color=(250, 250, 250)):
    """
    Adds a representation of the recognizable objects in a frame to the image.
    :param show_img: the base image.
    :param objects_in_frames: a list of the recognizable objects in the frame the image is from.
    :param detection_sign: are the objects detected.
    :param texts: the texts for the objects.
    :param text_color: the color for the text.
    :return: the image with the objects.
    """
    for i, obj in enumerate(objects_in_frames):
        if detection_sign and obj.x != 0 and obj.y != 0:
            show_img = cv2.circle(show_img, (int(obj.x), int(obj.y)), 15, (0, 0, 0), 3)
            if texts and texts[i]:
                show_img = cv2.putText(show_img, texts[i], (int(obj.x), int(obj.y)),
                                       cv2.FONT_HERSHEY_DUPLEX, 1, text_color, 2, cv2.LINE_AA)

    return show_img


def display_frames(balloon, drones, left_cam, right_cam, borders):
    """
    Prepares the frames of both cameras to be displayed.
    :param balloon: the RecognizableObject of the balloon.
    :param drones: a list of the Drones.
    :param left_cam: the left camera's Camera.
    :param right_cam: the right camera's Camera.
    :param borders: the Borders.
    :return: the images of the frames form both camera's with the relevant objects on it.
    """
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
