import numpy as np
import cv2


class XYDisplay:
    """
    A class for a 2D display of the XY plain of the game.
    """
    NUM_PIXELS_X = 512
    NUM_PIXELS_Y = 512
    MARGINS = 30
    GRID_DIFF = 10

    @staticmethod
    def get_xy_display(borders, balloon, drones, obstacle=None):
        """
        Returns the 2 dimensional display of the xy plain with all the relevant objects.
        :param borders: the Borders of the game.
        :param balloon: the RecognizableObject of the balloon.
        :param drones: a list of the Drones in play.
        :param obstacle: an Obstacle one of the drones is avoiding (if exists).
        :return: the xy display.
        """
        xy_display = np.zeros((XYDisplay.NUM_PIXELS_X, XYDisplay.NUM_PIXELS_Y, 3), np.uint8)
        x_lower_limit, x_upper_limit = np.min(borders.coordinates[:, 0]) - XYDisplay.MARGINS, np.max(
            borders.coordinates[:, 0]) + XYDisplay.MARGINS
        y_lower_limit, y_upper_limit = np.min(borders.coordinates[:, 1]) - XYDisplay.MARGINS, np.max(
            borders.coordinates[:, 1]) + XYDisplay.MARGINS
        limits = (x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit)
        recognizable_object = [balloon] + [drone.recognizable_object for drone in drones]

        if borders.is_set:
            borders_color = (0, 240, 0)  # green

            xy_display = XYDisplay._draw_grid(xy_display, limits)

            # calculates coordinates in pixels
            for i, recognizable_object in enumerate(recognizable_object):
                xy_display = XYDisplay._add_2d_object(recognizable_object.x, recognizable_object.y,
                                                      recognizable_object.text_colors,
                                                      recognizable_object.name, xy_display, limits)

                if not borders.in_borders(recognizable_object):
                    borders_color = (0, 0, 240)

            # draws destinations
            for i, drone in enumerate(drones):
                if np.any(drone.dest_coords):
                    xy_display = XYDisplay._add_2d_object(drone.dest_coords[0], drone.dest_coords[1],
                                                          (186, 85, 211), "dest" + str(i + 1), xy_display, limits)

            xy_display = XYDisplay._draw_borders(xy_display, borders, borders_color, limits)

        if obstacle:
            xy_display = XYDisplay._draw_obstacle(xy_display, obstacle, (255, 255, 255), limits)

        return xy_display

    @staticmethod
    def _add_2d_object(x, y, color, name, xy_display, limits, radius=15, circle_thickness=3,
                       text_shift=17, font_scale=0.5,
                       text_thickness=2):
        """
        Adds an object to the 2D display.
        :param x: the x coordinate of the object.
        :param y: the x coordinate of the object.
        :param color: the color for the object on the display.
        :param name: the name of the object.
        :param xy_display: the display.
        :param limits: the limits of the display.
        :param radius: the radius of the circle representing the object on the display.
        :param circle_thickness: the thickness of the circle.
        :param text_shift: the shift of the text from the circle.
        :param font_scale: the font scale of the name.
        :param text_thickness: the thickness if the name
        :return: the xy display with the object (as a circle).
        """
        x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit = limits
        x_pix, y_pix = XYDisplay._coor_to_pix(x, y, x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit)

        # object is inside limits of xy image
        if 0 <= x_pix <= XYDisplay.NUM_PIXELS_X and 0 <= y_pix <= XYDisplay.NUM_PIXELS_Y:
            xy_display = cv2.circle(xy_display, (x_pix, y_pix), radius, color, circle_thickness)
            if name:
                xy_display = cv2.putText(xy_display, name, (x_pix + text_shift, y_pix),
                                         cv2.FONT_HERSHEY_DUPLEX, font_scale, color, text_thickness, cv2.LINE_AA)
        return xy_display

    @staticmethod
    def _coor_to_pix(x, y, x_low_limit, x_upper_limit, y_low_limit, y_upper_limit):
        """
        Calculates the pixel location of given coordinates.
        :param x: the x coordinate.
        :param y:  the y coordinate.
        :param x_low_limit: the lower x limit of the display.
        :param x_upper_limit: the upper x limit of the display.
        :param y_low_limit: the lower y limit of the display.
        :param y_upper_limit: the upper y limit of the display.
        :return: the x,y coordinates in pixels.
        """
        x_pix = int(((x - x_low_limit) / (x_upper_limit - x_low_limit)) * XYDisplay.NUM_PIXELS_X)
        y_pix = abs(XYDisplay.NUM_PIXELS_Y - int(((y - y_low_limit) / (y_upper_limit - y_low_limit))
                                                 * XYDisplay.NUM_PIXELS_Y))

        return x_pix, y_pix

    @staticmethod
    def _draw_grid(img, limits):
        """
        Draws a grid on the display.
        :param img: the display.
        :param limits: the limits of the display.
        :return: the display with a grid.
        """
        x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit = limits
        pix_in_cm_x = XYDisplay.NUM_PIXELS_X / (x_upper_limit - x_lower_limit)
        pix_in_cm_y = XYDisplay.NUM_PIXELS_Y / (y_upper_limit - y_lower_limit)
        grid_length_x = int(XYDisplay.GRID_DIFF * pix_in_cm_x)
        grid_length_y = int(XYDisplay.GRID_DIFF * pix_in_cm_y)

        # draws grid on frame
        for i in range(grid_length_x, XYDisplay.NUM_PIXELS_X, grid_length_x):
            img = cv2.line(img, (i, 0), (i, XYDisplay.NUM_PIXELS_Y), (211, 211, 211), 1, 1)  # horizontal

        for i in range(grid_length_y, XYDisplay.NUM_PIXELS_Y, grid_length_y):
            img = cv2.line(img, (0, i), (XYDisplay.NUM_PIXELS_X, i), (211, 211, 211), 1, 1)  # vertical

        return img

    @staticmethod
    def _draw_quad(img, coordinates, color, limits):
        """
        Draws a quadrangle on the display.
        :param img: the display.
        :param coordinates: the coordinates of the corners of the quadrangle.
        :param color: the color of the quadrangle.
        :param limits: the limits of the display.
        :return: the display with the quadrangle.
        """
        x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit = limits
        corners = [0, 1, 3, 2]
        for i, cor in enumerate(corners):
            cor_next = corners[(i + 1) % len(corners)]
            img = cv2.line(img,
                           (XYDisplay._coor_to_pix(coordinates[cor][0], coordinates[cor][1],
                                                   x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit)),
                           (XYDisplay._coor_to_pix(coordinates[cor_next][0], coordinates[cor_next][1],
                                                   x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit)),
                           color,
                           thickness=2)

        return img

    @staticmethod
    def _draw_borders(img, borders, borders_color, limits):
        """
        Draws the border on the display.
        :param img: the display.
        :param borders: the Borders.
        :param borders_color: the color for the borders.
        :param limits: the limits of the display.
        :return: the display with the borders.
        """
        return XYDisplay._draw_quad(img, borders.coordinates, borders_color, limits)

    @staticmethod
    def _draw_obstacle(img, obstacle, obstacle_color, limits):
        """
        Draws an obstacle on the display.
        :param img: the display.
        :param obstacle: the Obstacle.
        :param obstacle_color: the color for the obstacle.
        :param limits: the limits of the display.
        :return: the display with the obstacle.
        """
        return XYDisplay._draw_quad(img, obstacle.coordinates, obstacle_color, limits)
