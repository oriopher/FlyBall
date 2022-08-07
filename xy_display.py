import numpy as np
import cv2

NUM_PIXELS_X = 512
NUM_PIXELS_Y = 512
MARGINS = 30
GRID_DIFF = 10


def add_2d_object(x, y, color, name, xy_display, limits, radius=15, circle_thickness=3,
                  text_shift=17, font_scale=0.5,
                  text_thickness=2):
    x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit  = limits
    x_pix, y_pix = coor_to_pix(x, y, x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit)

    # object is inside limits of xy image
    if x_pix >= 0 and y_pix >= 0:
        xy_display = cv2.circle(xy_display, (x_pix, y_pix), radius, color, circle_thickness)
        if name:
            xy_display = cv2.putText(xy_display, name, (x_pix + text_shift, y_pix),
                                        cv2.FONT_HERSHEY_DUPLEX, font_scale, color, text_thickness, cv2.LINE_AA)
    return xy_display


def get_xy_display(borders, recognizable_objects, obstacle = None):
    xy_display = np.zeros((NUM_PIXELS_X, NUM_PIXELS_Y, 3), np.uint8)
    x_lower_limit, x_upper_limit = np.min(borders.coordinates[:, 0]) - MARGINS, np.max(borders.coordinates[:, 0]) + MARGINS
    y_lower_limit, y_upper_limit = np.min(borders.coordinates[:, 1]) - MARGINS, np.max(borders.coordinates[:, 1]) + MARGINS
    limits = (x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit)
    
    if borders.set_borders:
        borders_color = (0, 240, 0)  # green

        xy_display = draw_grid(xy_display, limits)

        # calculats coordinates in pixels
        for i, recognizable_object in enumerate(recognizable_objects):
            xy_display = add_2d_object(borders, recognizable_object.x, recognizable_object.y, recognizable_object.text_colors,
                                       recognizable_object.name, xy_display)

            if borders.in_borders(recognizable_object):
                borders_color = (240, 0, 0)

            # draws destinations
            if i > 0 and np.any(recognizable_object.dest_coords):
                xy_display = add_2d_object(recognizable_object.dest_coords[0], recognizable_object.dest_coords[1], 
                                        (186, 85, 211), "dest"+str(i), xy_display, limits)
   
        xy_display = draw_borders(xy_display, borders, borders_color, limits)

    if obstacle:
        xy_display = draw_obstacle(xy_display, obstacle, (255, 255, 255), limits)    

    return xy_display


def coor_to_pix(x, y, x_low_limit, x_upper_limit, y_low_limit, y_upper_limit):
    x_pix = int(((x - x_low_limit) / (x_upper_limit - x_low_limit)) * NUM_PIXELS_X)
    y_pix = abs(NUM_PIXELS_Y - int(((y - y_low_limit) / (y_upper_limit - y_low_limit)) * NUM_PIXELS_Y))

    return x_pix, y_pix

def draw_grid(img, limits):
    x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit  = limits        
    pix_in_cm_x = NUM_PIXELS_X / (x_upper_limit - x_lower_limit)
    pix_in_cm_y = NUM_PIXELS_Y / (y_upper_limit - y_lower_limit)
    grid_length_x = int(GRID_DIFF * pix_in_cm_x)
    grid_length_y = int(GRID_DIFF * pix_in_cm_y)

    # draws grid on frame
    for i in range(grid_length_x, NUM_PIXELS_X, grid_length_x):
        img = cv2.line(img, (i, 0), (i, NUM_PIXELS_Y), (211, 211, 211), 1, 1) # horizontal

    for i in range(grid_length_y, NUM_PIXELS_Y, grid_length_y):
        img = cv2.line(img, (0, i),(NUM_PIXELS_X, i), (211, 211, 211), 1, 1) # vertical

    return img


def draw_quad(img, coordinates, color, limits):
    x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit  = limits        
    corners = [0, 1, 3, 2]
    for i, cor in enumerate(corners):
        cor_next = corners[(i + 1) % len(corners)]
        img = cv2.line(img,
                                (coor_to_pix(coordinates[cor][0], coordinates[cor][1], x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit)),
                                (coor_to_pix(coordinates[cor_next][0], coordinates[cor_next][1], x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit)),
                                color,
                                thickness=2)   

    return img                                 


def draw_borders(img, borders, borders_color, limits):
    return draw_quad(img, borders.coordinates, borders_color, limits)

def draw_obstacle(img, obstacle, obstacle_color, limits):
    return draw_quad(img, obstacle.coordinates, obstacle_color, limits)


