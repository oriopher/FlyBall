import numpy as np
from borders import Borders
import cv2

NUM_PIXELS_X = 512
NUM_PIXELS_Y = 512


def add_2d_object(borders, x, y, color, name, xy_display, radius=15, circle_thickness=3,
                  text_shift=17, font_scale=0.5,
                  text_thickness=2):
    x_lower_limit, x_upper_limit = np.min(borders.coordinates[:, 0]), np.max(borders.coordinates[:, 0])
    y_lower_limit, y_upper_limit = np.min(borders.coordinates[:, 1]), np.max(borders.coordinates[:, 1])
    x_pix, y_pix = x_coor_to_pix(x, x_lower_limit, x_upper_limit), y_coor_to_pix(y, y_lower_limit, y_upper_limit)
    xy_display = cv2.circle(xy_display, (x_pix, y_pix), radius, color, circle_thickness)
    if name:
        xy_display = cv2.putText(xy_display, name, (x_pix + text_shift, y_pix),
                                 cv2.FONT_HERSHEY_DUPLEX, font_scale, color, text_thickness, cv2.LINE_AA)
    return xy_display


def draw_xy_display(borders: Borders, recognizable_objects, x_pred_phys=None, y_pred_phys=None):
    xy_display = np.zeros((NUM_PIXELS_X, NUM_PIXELS_Y, 3), np.uint8)
    x_lower_limit, x_upper_limit = np.min(borders.coordinates[:, 0]), np.max(borders.coordinates[:, 0])
    y_lower_limit, y_upper_limit = np.min(borders.coordinates[:, 1]), np.max(borders.coordinates[:, 1])
    borders_color = (0, 240, 0)  # green

    if borders.set_borders:
        # calc coordinates in pixels
        for recognizable_object in recognizable_objects:
            xy_display = add_2d_object(borders, recognizable_object.x, recognizable_object.y, recognizable_object.color,
                                       recognizable_object.name, xy_display)
        xy_display = add_2d_object(borders, x_pred_phys, y_pred_phys, (186, 85, 211), None, xy_display)
        corners = [0, 1, 3, 2]
        for i in corners:
            i_next = (i + 1) % len(corners)
            xy_display = cv2.line(xy_display,
                                  (x_coor_to_pix(borders.coordinates[i][0], x_lower_limit, x_upper_limit),
                                   y_coor_to_pix(borders.coordinates[i][1], y_lower_limit, y_upper_limit)),
                                  (x_coor_to_pix(borders.coordinates[i_next][0], x_lower_limit, x_upper_limit),
                                   y_coor_to_pix(borders.coordinates[i_next][1], y_lower_limit, y_upper_limit)),
                                  borders_color,
                                  thickness=2)
    cv2.imshow('XY Display', xy_display)


def x_coor_to_pix(x, x_low_limit, x_upper_limit):
    return int(((x - x_low_limit) / (x_upper_limit - x_low_limit)) * NUM_PIXELS_X)


def y_coor_to_pix(y, y_low_limit, y_upper_limit):
    return abs(NUM_PIXELS_Y - int(((y - y_low_limit) / (y_upper_limit - y_low_limit)) * NUM_PIXELS_Y))
