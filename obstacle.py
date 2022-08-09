from scipy.sparse import csr_matrix
from scipy.sparse.csgraph._shortest_path import dijkstra
from scipy.spatial import distance_matrix

from common import calc_linear_eq
from quadrangle import Quadrangle
import numpy as np

"""
###############################
#### a-------------------d ####
#### |-------------------| ####
#### |-------------------| ####
#### |-------------------| ####
#### |-------------------| ####
#### b-------------------c ####
###############################
"""


class Obstacle:
    EPSILON = 1e-3
    EXIT_MARGIN = 100
    MARGINS = 1
    MARGINS_END = 35
    MARGINS_START = 30
    MARGINS_SIDES = 30 # start must be graeter than sides

    def __init__(self, drone, left_cam):
        self.start = (drone.x, drone.y)
        self.end = (drone.dest_coords[0], drone.dest_coords[1])
        # print("obstacle start, end = ", self.start, self.end)
        self._preparation_dest = (0, 0)
        self._quad = Quadrangle(self._calc_corners(), left_cam)

    @property
    def coordinates(self):
        return self._quad.coordinates

    def _closest_corners(self, p1, p2, margin1, margin2):
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
        corner1, corner2, self._preparation_dest = self._closest_corners(self.end, self.start, self.MARGINS_END, self.MARGINS_SIDES)
        corner3, corner4, _ = self._closest_corners(self.start, self.end, self.MARGINS_START, self.MARGINS_SIDES)

        return corner1, corner2, corner3, corner4

    # checks if the drone is inside the obstacle
    def inside_obstacle(self, drone):
        return self.coord_in_obstacle(drone.x, drone.y)

    # checks if the given coordinate is inside the obstacle
    def coord_in_obstacle(self, x, y):
        return self._quad.coordinate_in_quadrangle(x, y)

    # draws the obstacle on left frame
    def draw_obstacle(self, show_img, recognizable_object, color_in=(240, 0, 0), color_out=(0, 240, 0)):
        color = color_out

        if self.inside_obstacle(recognizable_object):
            color = color_in
        show_img = self._quad.draw_quadrangle(show_img, color)

        return show_img

    def bypass_obstacle_coordinates(self, source, target):
        if self.coord_in_obstacle(*source):
            # print("EXIT!!!")
            return self._get_exit_dest(*source, self.EXIT_MARGIN)

        if self.coord_in_obstacle(*target):
            new_target = self._get_exit_dest(*target)
            # print("dest in obstacle, new dest: ", target[0], target[1])
            return self.bypass_obstacle_coordinates(source, new_target)

        obstacle_distances = self._get_corners_reachable_distances()
        target_vertices = np.vstack([self.coordinates, target])
        target_vertices_distance = np.insert(self._get_reachable_distances(source, target_vertices), 0, 0)
        reachable_to_target_distances = self._get_reachable_distances(target, self.coordinates)
        corners_distances = np.hstack([np.zeros((4, 1)), obstacle_distances, np.reshape(reachable_to_target_distances, (-1, 1))])
        reachable_distances = np.vstack([target_vertices_distance, corners_distances, np.zeros((1, 6))])
        path = self._get_shortest_path(reachable_distances)
        if not path:
            return source
        next_vertex = path[0]
        next_point = target_vertices[next_vertex - 1]
        # print("next point: ", next_point)
        return self._continue_line_to_distance(*source, *next_point, np.linalg.norm(np.array(source) - np.array(target)))[0]

    def _get_reachable_distances(self, point, vertices):
        distances = np.linalg.norm(vertices - point, axis=1)
        reachable = 1 - np.apply_along_axis(lambda point2: self._quad.cross_quadrangle(point, point2), 1, vertices)
        return reachable * distances

    def _get_corners_reachable_distances(self):
        obstacle_distances = distance_matrix(self.coordinates, self.coordinates)
        for i, j in [(0, 3), (1, 2)]:
            obstacle_distances[i][j] = 0
            obstacle_distances[j][i] = 0
        return obstacle_distances

    def get_preparation_dest(self):
        return self._preparation_dest

    # returns the nearest exit point from obstacle
    def _get_exit_dest(self, x, y, exit_dist=MARGINS):
        curves = np.zeros((4, 2))

        if self.coord_in_obstacle(x, y):
            # finds rectangle curves
            for i, cor in enumerate(self._quad.CORNERS):
                cor_next = self._quad.CORNERS[(i + 1) % len(self._quad.CORNERS)]
                curves[i] = calc_linear_eq(self.coordinates[cor], self.coordinates[cor_next])

            distances = self._calc_distances_to_curves(curves, x, y)

            # index of closest curve     
            idx = np.argmin(distances)
            return self._calc_coor(curves[idx][0], curves[idx][1], x, y, exit_dist)

        return 0, 0

    # calculates coordinates in EXIT_DISTANCE from given curve 
    def _calc_coor(self, m, b, x, y, distance):
        if np.isnan(m):
            s = 0 if b > x else 1
            return b + distance * (-1)**s, y
        elif m == 0:
            s = 0 if b > y else 1
            return x, b + distance * (-1)**s

        sgn = np.sign(y - m * x - b)

        # x,y is below the closest curve
        if sgn < 0:
            return self._calc_coor_above(m, b, x, y, distance)

        # x,y is above the closest curve
        return self._calc_coor_below(m, b, x, y, distance)

    def _calc_distances_to_curves(self, curves, x, y):
        distances = np.array([])
        for i, curve in enumerate(curves):
            if curve[0] is None or np.isnan(curve[0]):
                distances = np.append(distances, abs(x - self.coordinates[i][0]))
            else:
                distances = np.append(distances, abs(curve[0] * x - y + curve[1]) / np.sqrt(curve[0] ** 2 + 1))
        return distances

    @staticmethod
    def _continue_line_to_distance(x1, y1, x2, y2, r):
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

    # returns the coor above given line in a given distance
    @staticmethod
    def _calc_coor_above(m, b, x, y, distance):
        c = y - (-1/m) * x
        x_coor = (c - b - distance * np.sqrt(m**2 + 1)) / (m + 1/m)
        y_coor = (-1/m) * x_coor + c

        return x_coor, y_coor

    # return the coor below the given line in a given distance
    @staticmethod
    def _calc_coor_below(m, b, x, y, distance):
        c = y - (-1/m) * x
        x_coor = (c - b + distance * np.sqrt(m**2 + 1)) / (m + 1/m)
        y_coor = (-1/m) * x_coor + c

        return x_coor, y_coor

    @staticmethod
    def _get_shortest_path(reachable_distances):
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
    def _solve_quadratic(a, b, c, s):
        delta = b ** 2 - 4 * a * c
        # if delta < 0:
            # print("quadratic error: a = {:.2f}, b = {:.2f}, c = {:.2f}, b^2-4ac = {:.2f}".format(a, b, c, delta))
        return (-b + np.sqrt(delta) * ((-1) ** s)) / (2 * a)
