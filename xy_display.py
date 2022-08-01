import numpy as np
import cv2

NUM_PIXELS_X = 512
NUM_PIXELS_Y = 512
MARGINS = 20
GRID_DIFF = 10


def add_2d_object(borders, x, y, color, name, xy_display, radius=15, circle_thickness=3,
                  text_shift=17, font_scale=0.5,
                  text_thickness=2):
    x_lower_limit, x_upper_limit = np.min(borders.coordinates[:, 0]), np.max(borders.coordinates[:, 0])
    y_lower_limit, y_upper_limit = np.min(borders.coordinates[:, 1]), np.max(borders.coordinates[:, 1])
    x_pix, y_pix = coor_to_pix(x, y, x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit)
    xy_display = cv2.circle(xy_display, (x_pix, y_pix), radius, color, circle_thickness)
    if name:
        xy_display = cv2.putText(xy_display, name, (x_pix + text_shift, y_pix),
                                 cv2.FONT_HERSHEY_DUPLEX, font_scale, color, text_thickness, cv2.LINE_AA)
    return xy_display


def draw_xy_display(borders, recognizable_objects):
    xy_display = np.zeros((NUM_PIXELS_X, NUM_PIXELS_Y, 3), np.uint8)
    x_lower_limit, x_upper_limit = np.min(borders.coordinates[:, 0]) - MARGINS, np.max(borders.coordinates[:, 0]) + MARGINS
    y_lower_limit, y_upper_limit = np.min(borders.coordinates[:, 1]) - MARGINS, np.max(borders.coordinates[:, 1]) + MARGINS

    if borders.set_borders:
        borders_color = (0, 240, 0)  # green

        pix_in_cm_x, pix_in_cm_y = NUM_PIXELS_X / (x_upper_limit - x_lower_limit), NUM_PIXELS_Y / (y_upper_limit - y_lower_limit)
        grid_length_x ,grid_length_y = int(GRID_DIFF * pix_in_cm_x), int(GRID_DIFF * pix_in_cm_y)

        # drawing grid on frame
        for i in range(grid_length_x, NUM_PIXELS_X, grid_length_x):
            xy_display = cv2.line(xy_display, (i, 0), (i, NUM_PIXELS_Y), (211, 211, 211), 1, 1) # horizontal

        for i in range(grid_length_y, NUM_PIXELS_Y, grid_length_y):
            xy_display = cv2.line(xy_display, (0, i),(NUM_PIXELS_X, i), (211, 211, 211), 1, 1) # vertical    

        # calc coordinates in pixels
        for i, recognizable_object in enumerate(recognizable_objects):
            xy_display = add_2d_object(borders, recognizable_object.x, recognizable_object.y, recognizable_object.text_colors,
                                       recognizable_object.name, xy_display)

            if borders.in_borders(recognizable_object):
                borders_color = (240, 0, 0)

            # drawing predictions
            if i > 0:
                if np.any(recognizable_object.dest_coords):
                    xy_display = add_2d_object(borders, recognizable_object.dest_coords[0], recognizable_object.dest_coords[1], (186, 85, 211), None, xy_display)                

        # drawing borders
        corners = [0, 1, 3, 2]
        for i, cor in enumerate(corners):
            cor_next = corners[(i + 1) % len(corners)]
            xy_display = cv2.line(xy_display,
                                  (coor_to_pix(borders.coordinates[cor][0], borders.coordinates[cor][1], x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit)),
                                  (coor_to_pix(borders.coordinates[cor_next][0], borders.coordinates[cor_next][1], x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit)),
                                  borders_color,
                                  thickness=2)

    cv2.imshow('XY Display', xy_display)


def coor_to_pix(x, y, x_low_limit, x_upper_limit, y_low_limit, y_upper_limit):
    x_pix = int(((x - x_low_limit) / (x_upper_limit - x_low_limit)) * NUM_PIXELS_X)
    y_pix = abs(NUM_PIXELS_Y - int(((y - y_low_limit) / (y_upper_limit - y_low_limit)) * NUM_PIXELS_Y))

    return x_pix, y_pix