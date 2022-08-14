from scipy.sparse import csr_matrix
from scipy.sparse.csgraph._shortest_path import dijkstra
from scipy.spatial import distance_matrix

from utils.common import calc_linear_eq
from quadrangles.quadrangle import Quadrangle
import numpy as np

"""
###############################
#### a-------------------d ####
#### |-------------------| ####
#### |-------------------| ####
#### |--drone----dest----| ####
#### |-------------------| ####
#### |-------------------| ####
#### b-------------------c ####
###############################
"""


class Obstacle:
    """
    A class representing a 2 dimensional obstacle around a drone and its destination.
    """
    EPSILON = 1e-3
    EXIT_MARGIN = 100
    MARGINS = 1
    MARGINS_END = 35
    MARGINS_START = 30
    MARGINS_SIDES = 30  # start must be greater than sides

    def __init__(self, drone, left_cam):
        """
        Initializes the obstacle.
        :param drone: the drone the obstacle is constructed around.
        :param left_cam: the left cameras Camera.
        """
        self.start = (drone.x, drone.y)
        self.end = (drone.dest_coords[0], drone.dest_coords[1])
        self._preparation_dest = (0, 0)
        self._quad = Quadrangle(self._calc_corners(), left_cam)

    @property
    def coordinates(self):
        """
        :return: the coordinated of the corners of the obstacle.
        """
        return self._quad.coordinates

    def get_preparation_dest(self):
        """
        :return: the preparation location for a drone outside the obstacle
        (MARGINS cms outside of the obstacle on the continuation of the line
        between the constructing drone to its dest on the closer side to the destination).
        """
        return self._preparation_dest

    def inside_obstacle(self, drone):
        """
        Checks if a drone is inside the obstacle.
        :param drone: a RecognizableObject of the drone.
        :return: True if the drone is inside the obstacle, and False otherwise.
        """
        return self.coord_in_obstacle(drone.x, drone.y)

    def coord_in_obstacle(self, x, y):
        """
        Check if a given point is inside the obstacle.
        :param x: the x coordinate of the point.
        :param y: the x coordinate of the point.
        :return: True if the point is inside the obstacle, and False otherwise.
        """
        return self._quad.point_in_quadrangle(x, y)

    def draw_obstacle(self, show_img, recognizable_object, color_in=(240, 0, 0), color_out=(0, 240, 0)):
        """
        Draws the obstacle (on the left camera frame).
        :param show_img: the frame of the left camera.
        :param recognizable_object: the RecognizableObject of the drone.
        :param color_in: the color of the obstacle if the drone is inside of the obstacle.
        :param color_out: the color of the obstacle if the drone is outside of the obstacle.
        :return: the frame with the obstacle.
        """
        color = color_out

        if self.inside_obstacle(recognizable_object):
            color = color_in
        show_img = self._quad.draw_quadrangle(show_img, color)

        return show_img

    def bypass_obstacle_coordinates(self, source, target):
        """
        Calculates the x,y coordinates destination for a path from a source to a target bypassing the obstacle.
        :param source: a source location for the path.
        :param target: a target location for the path.
        :return: the new destination for the next point on the path.
        """
        # Make the drone exit the obstacle if its inside the obstacle.
        if self.coord_in_obstacle(*source):
            return self._get_exit_dest(*source, self.EXIT_MARGIN)

        # Move the target to be outside the obstacle if its inside it.
        if self.coord_in_obstacle(*target):
            new_target = self._get_exit_dest(*target)
            # print("dest in obstacle, new dest: ", target[0], target[1])
            return self.bypass_obstacle_coordinates(source, new_target)

        # Construct a graph of paths from the source to the target via the obstacles corners
        # without crossing the obstacle itself. The edges weights are the distances between the vertices.
        obstacle_distances = self._get_corners_reachable_distances()
        target_vertices = np.vstack([self.coordinates, target])
        target_vertices_distance = np.insert(self._get_reachable_distances(source, target_vertices), 0, 0)
        reachable_to_target_distances = self._get_reachable_distances(target, self.coordinates)
        corners_distances = np.hstack(
            [np.zeros((4, 1)), obstacle_distances, np.reshape(reachable_to_target_distances, (-1, 1))])
        reachable_distances = np.vstack([target_vertices_distance, corners_distances, np.zeros((1, 6))])

        # Determine the first vertex on the shortest path from the source to the target.
        path = self._get_shortest_path(reachable_distances)
        if not path:
            return source
        next_vertex = path[0]
        next_point = target_vertices[next_vertex - 1]
        return self._continue_line_to_distance(*source, *next_point, np.linalg.norm(np.array(source) - np.array(target)))[0]

    def _closest_corners(self, p1, p2, margin1, margin2):
        """
        Calculates 2 corners of the obstacle from two inputted points.
        A sketch depicting the corners calculation (* - the 2 calculated corners, + - margin1, ! - margin2):
        ###############################
        #### .-------------------* ####
        #### |-------------------! ####
        #### |-------------------! ####
        #### |--p2---------p1++++| ####
        #### |-------------------| ####
        #### |-------------------| ####
        #### .-------------------* ####
        ###############################
        :param p1: the first point the obstacle is constructed around
                   (the corners calculated are the closest to this point).
        :param p2: the second point the obstacle is constructed around.
        :param margin1: the margin between p1 and the middle of the edge between the 2 corners calculated.
        :param margin2: the margin between each corner and the middle of the edge between them.
        :return: the 2 corners depicted and the middle point between them.
        """
        _, mid = self._continue_line_to_distance(*p1, *p2, margin1)

        # vertical track
        if p1[0] == p2[0]:
            corner1 = (mid[0] + margin2, mid[1])
            corner2 = (mid[0] - margin2, mid[1])

        # horizontal track
        elif np.abs(p1[1] - p2[1]) < self.EPSILON:
            corner1 = (mid[0], mid[1] + margin2)
            corner2 = (mid[0], mid[1] - margin2)

        else:
            a = -(p2[0] - p1[0])/(p2[1] - p1[1])  # slope of perpendicular line
            b = mid[1] - a * mid[0]
            x2 = mid[0] + 1
            y2 = a*x2 + b
            corner1, corner2 = self._continue_line_to_distance(*mid, x2, y2, margin2)

        return corner1, corner2, mid

    def _calc_corners(self):
        """
        calculates the corners oft the obstacle.
        :return:
        """
        corner1, corner2, self._preparation_dest = self._closest_corners(self.end, self.start, self.MARGINS_END, self.MARGINS_SIDES)
        corner3, corner4, _ = self._closest_corners(self.start, self.end, self.MARGINS_START, self.MARGINS_SIDES)

        return corner1, corner2, corner3, corner4

    def _get_reachable_distances(self, point, vertices):
        """
        Gets the distances from an inputted point to each vertex in an inputted list.
        If the vertex is not reachable without crossing the obstacle the weight is set to 0.
        :param point: the source point to the list of vertices,
        :param vertices: a list of vertices (numpy array).
        :return: a numpy array of the distances from the inputted point to the reachable vertices.
        """
        distances = np.linalg.norm(vertices - point, axis=1)
        reachable = 1 - np.apply_along_axis(lambda point2: self._quad.cross_quadrangle(point, point2), 1, vertices)
        return reachable * distances

    def _get_corners_reachable_distances(self):
        """
        Gets the distance matrix between the corners of the obstacle.
        If the path between 2 corners is not reachable without crossing the obstacle the weight is set to 0.
        :return: the distance matrix.
        """
        obstacle_distances = distance_matrix(self.coordinates, self.coordinates)
        for i, j in [(0, 3), (1, 2)]:
            obstacle_distances[i][j] = 0
            obstacle_distances[j][i] = 0
        return obstacle_distances

    def _get_exit_dest(self, x, y, exit_dist=MARGINS):
        """
        Gets the nearest exit point from obstacle for an inputted point  (in the wanted distance from the obstacle).
        :param x: the x coordinate of the point.
        :param y: the y coordinate of the point.
        :param exit_dist: the distance of the calculated point from the obstacle.
        :return: a new point outside the obstacle exiting the nearest side and distanced the wanted amount.
        """
        curves = np.zeros((4, 2))

        if self.coord_in_obstacle(x, y):
            # finds rectangle curves
            for i, cor in enumerate(self._quad.CORNERS):
                cor_next = self._quad.CORNERS[(i + 1) % len(self._quad.CORNERS)]
                curves[i] = calc_linear_eq(self.coordinates[cor], self.coordinates[cor_next])

            distances = self._calc_distances_to_curves(curves, x, y)

            # index of closest curve     
            idx = np.argmin(distances)
            return self._exit_coord_via_line(curves[idx][0], curves[idx][1], x, y, exit_dist)

        return 0, 0

    def _calc_distances_to_curves(self, curves, x, y):
        """
        Calculates the distances of a point to a list of curves.
        :param curves: the list of curves.
        :param x: the x coordinate of the point.
        :param y: the y coordinate of the point.
        :return: a list of the distances.
        """
        distances = np.array([])
        for i, curve in enumerate(curves):
            if curve[0] is None or np.isnan(curve[0]):
                distances = np.append(distances, abs(x - self.coordinates[i][0]))
            else:
                distances = np.append(distances, abs(curve[0] * x - y + curve[1]) / np.sqrt(curve[0] ** 2 + 1))
        return distances

    def _exit_coord_via_line(self, a, b, x, y, distance):
        """
        Calculates a point that is distanced by an inputted amount from an inputted line
        on the continuation from an inputted point to the line (perpendicular).
        :param a: the slope of the line.
        :param b: the free coefficient of the line.
        :param x: the x coordinate of the point.
        :param y: the y coordinate of the point.
        :param distance: the distance of the calculated point from the line.
        :return: the point on the other side of the line with the wanted distance.
        """
        if np.isnan(a):
            s = 0 if b > x else 1
            return b + distance * (-1)**s, y
        elif a == 0:
            s = 0 if b > y else 1
            return x, b + distance * (-1)**s

        sgn = np.sign(y - a * x - b)

        # x,y is below the closest curve
        if sgn < 0:
            return self._calc_exit_coor_above(a, b, x, y, distance)

        # x,y is above the closest curve
        return self._calc_exit_coor_below(a, b, x, y, distance)

    # returns the coor above given line in a given distance
    @staticmethod
    def _calc_exit_coor_above(a, b, x, y, distance):
        """
        Calculated an exit coordinate (line in _exit_coord_via_line) above the line.
        :param a: the slope of the line.
        :param b: the free coefficient of the line.
        :param x: the x coordinate of the point.
        :param y: the y coordinate of the point.
        :param distance: the distance of the calculated point from the line.
        :return: the point above the line with the wanted distance.
        """
        c = y - (-1 / a) * x
        x_coor = (c - b - distance * np.sqrt(a ** 2 + 1)) / (a + 1 / a)
        y_coor = (-1 / a) * x_coor + c

        return x_coor, y_coor

    # return the coor below the given line in a given distance
    @staticmethod
    def _calc_exit_coor_below(a, b, x, y, distance):
        """
        Calculated an exit coordinate (line in _exit_coord_via_line) bellow the line.
        :param a: the slope of the line.
        :param b: the free coefficient of the line.
        :param x: the x coordinate of the point.
        :param y: the y coordinate of the point.
        :param distance: the distance of the calculated point from the line.
        :return: the point bellow the line with the wanted distance.
        """
        c = y - (-1 / a) * x
        x_coor = (c - b + distance * np.sqrt(a ** 2 + 1)) / (a + 1 / a)
        y_coor = (-1 / a) * x_coor + c

        return x_coor, y_coor

    @staticmethod
    def _get_shortest_path(reachable_distances):
        """
        Gets the shortest path from the first vertex to last vertex in a weighted graph.
        :param reachable_distances: an adjacency matrix of the graph.
        :return: a list of the shortest path from the first vertex to the last (without the first vertex).
        """
        reachable_distances_graph = csr_matrix(reachable_distances)
        dist_matrix, predecessors = dijkstra(csgraph=reachable_distances_graph, directed=True, indices=0,
                                             return_predecessors=True)
        target = len(predecessors) - 1
        vertex = target
        path = []
        while vertex != 0:
            path.insert(0, vertex)
            vertex = predecessors[vertex]
            if 0 > vertex or vertex > target:
                return []
        return path

    @staticmethod
    def _continue_line_to_distance(x1, y1, x2, y2, r):
        """
        Calculates a point that is on the line continuation from one point to a second point
        that is in a wanted distance from the first point.
        :param x1: the x coordinate of the first point.
        :param y1: the y coordinate of the first point.
        :param x2: the x coordinate of the second point.
        :param y2: the y coordinate of the second point.
        :param r: the wanted distance of the calculated point from the first point.
        :return: the continuation point with the wanted distance.
        """
        if np.abs(x1-x2) < Obstacle.EPSILON:
            s = 0 if y2 > y1 else 1
            return (x1, y1 + r * (-1)**s), (x1, y1 + r * (-1)**(1-s))
        a = (y1 - y2) / (x1 - x2)
        b = y1 - a*x1
        s = 0 if x2 > x1 else 1
        x3 = Obstacle._solve_quadratic(a**2 + 1, 2*(a*b - x1 - a*y1), y1**2+x1**2+b**2-2*b*y1-r**2, s)
        y3 = a*x3 + b
        x4 = Obstacle._solve_quadratic(a**2 + 1, 2*(a*b - x1 - a*y1), y1**2+x1**2+b**2-2*b*y1-r**2, 1-s)
        y4 = a*x4 + b
        return (x3, y3), (x4, y4)

    @staticmethod
    def _solve_quadratic(a, b, c, s):
        """
        Solves an inputted quadratic equation (a*x^2 + b*x + c) with the wanted sign of the square root part.
        :param a: the x^2 coefficient.
        :param b: the x coefficient.
        :param c: the free coefficient.
        :param s: representing the wanted sign of the square root part
                  (sqrt(b^2 -4ac)) in the solution (0 for + and 1 for -)
        :return: the solution of the quadratic equation with the wanted sign.
        """
        delta = b ** 2 - 4 * a * c
        return (-b + np.sqrt(delta) * ((-1) ** s)) / (2 * a)
