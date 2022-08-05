from scipy.sparse import csr_matrix
from scipy.sparse.csgraph._shortest_path import dijkstra
from scipy.spatial import distance_matrix

from common import calc_linear_eq
from drone import Drone
from quadrangle import Quadrangle
import numpy as np
import cv2

MARGINS = 25
EXIT_DIST = 10


class Obstacle:
    def __init__(self, drone, left_cam):
        self.start = (0, 0)
        self.end = (0, 0)
        self.quad = Quadrangle(self.calc_corners(), left_cam)

    @property
    def coordinates(self):
        return self.quad.coordinates

    def _calc_middles(self):
        x_middle = (self.start[0] + self.end[0]) / 2
        y_middle = (self.start[1] + self.end[1]) / 2

        # rectangle parallel to y axis
        if self.end[0] == self.start[0]:
            x_middle_low = x_middle- MARGINS
            x_middle_upper = x_middle + MARGINS
            y_middle_low = y_middle
            y_middle_upper = y_middle

        # rectangle parallel to x axis
        elif self.end[1] == self.start[1]:
            x_middle_low = x_middle
            x_middle_upper = x_middle
            y_middle_low = y_middle - MARGINS
            y_middle_upper = y_middle + MARGINS

        else:
            m_track = (self.start[1] - self.end[1]) / (self.start[0] - self.end[0])
            b_track = y_middle - m_track * x_middle

            x_middle_low, y_middle_low = self._calc_coor_below(m_track, b_track, x_middle, y_middle, MARGINS)
            x_middle_upper, y_middle_upper = self._calc_coor_above(m_track, b_track, x_middle, y_middle, MARGINS)

        return x_middle_low, y_middle_low, x_middle_upper, y_middle_upper


    # finds the vertexes of the rectangle
    def calc_corners(self):
        x_middle_low, y_middle_low, x_middle_upper, y_middle_upper = self._calc_middles()
        length = np.sqrt((self.start[0] - self.end[0]) ** 2 + (self.start[1] - self.end[1]) ** 2) + 2 * MARGINS
                
        # Horizontal rectangle
        if x_middle_upper == x_middle_low:
            a_x = x_middle_upper - (length / 2)
            a_y = y_middle_upper
            
            d_x = x_middle_upper + (length / 2)
            d_y = y_middle_upper
            
            b_x = x_middle_low - (length / 2)
            b_y = y_middle_low

            c_x = x_middle_low + (length / 2)
            c_y = y_middle_low
            
        # Vertical rectangle
        elif y_middle_upper == y_middle_low:
            a_y = y_middle_upper - (length / 2)
            a_x = x_middle_upper

            d_y = y_middle_upper + (length / 2)
            d_x = x_middle_upper

            b_y = y_middle_low - (length / 2)
            b_x = x_middle_low

            c_y = y_middle_low + (length / 2)
            c_x = x_middle_low
        
        # Slanted rectangle
        else:
            # Calculate slope of the side
            m = (x_middle_upper - x_middle_low) / (y_middle_low - y_middle_upper)
            
            # Calculate displacements along axes
            dx = (length / np.sqrt(1 + (m ** 2))) * 0.5
            dy = m * dx
            
            a_x = x_middle_upper - dx
            a_y = y_middle_upper - dy

            d_x = x_middle_upper + dx
            d_y = y_middle_upper + dy

            b_x = x_middle_low - dx
            b_y = y_middle_low - dy

            c_x = x_middle_low + dx
            c_y = y_middle_low + dy

        # saves rectangle's coordinates
        self.coordinates = [(c_x, c_y), (b_x, b_y), (d_x, d_y), (a_x, a_y)]

    # checks if the drone is inside the obstacle
    def inside_obstacle(self, drone):
        return self.coord_in_obsatcle(drone.x, drone.y)

    # checks if the given coordinate is inside the obstacle
    def coord_in_obsatcle(self, x, y):
        return self.quad.coordinate_in_quadrangle(x, y)

    # draws the obstacle on left frame
    def draw_obstacle(self, show_img, recognizable_object, color_in=(240, 0, 0), color_out=(0, 240, 0)):
        color = color_out

        if self.inside_obstacle(self, recognizable_object):
            color = color_in
        show_img = self.quad.draw_quadrangle(show_img, color)

        return show_img

    def bypass_obstacle_coordinates(self, source, target):

        obstacle_distances = self._get_corners_reachable_distances()
        target_vertices = self._get_reachable_distances(source, np.vstack([self.coordinates, target]))
        reachable_from_source_distances = np.append(0, target_vertices)
        reachable_to_target_distances = np.append(
            self._get_reachable_distances(target, np.vstack([source, self.coordinates])), 0)
        reachable_distances = np.vstack(
            [reachable_from_source_distances, np.hstack([(0, 0), obstacle_distances, (0, 0)]),
             reachable_to_target_distances])
        path = self._get_shortest_path(reachable_distances)
        if not path:
            return source
        next_vertex = path[0]
        next_point = target_vertices[next_vertex - 1]
        return self._continue_line_to_distance(*source, *target, np.linalg.norm(source - next_point))

    def _get_reachable_distances(self, point, vertices):
        distances = np.linalg.norm(vertices - point)
        reachable = 1 - np.apply_along_axis(lambda point2: self.quad.cross_quadrangle(point, point2), vertices, 1)
        return reachable * distances

    def _get_corners_reachable_distances(self):
        obstacle_distances = distance_matrix(self.coordinates, self.coordinates)
        for i, j in [(1, 3), (2, 4)]:
            obstacle_distances[i][j] = 0
            obstacle_distances[j][i] = 0
        return obstacle_distances

    def _get_preperation_dest(self):
        distances = np.array([])
        # calculates squared distance to every point of te rectangle
        for coordinate in self.coordinates:
            res = (self.end[0] - coordinate[0]) ** 2  + (self.end[1] - coordinate[1]) ** 2 
            np.append(distances, res)

        # finds 2 nearest points
        first_point = self.coordinates[np.argmin(distances)]
        distances[np.argmin(distances)] = distances[np.max(distances)] + 1
        second_point = self.coordinates[np.argmin(distances)]

        x_dest = first_point[0] + second_point[0] / 2
        y_dest = first_point[1] + second_point[1] / 2

        return x_dest, y_dest

    # returns the nearest exit point from obsatacle
    def _get_exit_dest(self, x, y):
        corners = [0, 1, 3, 2]
        curves = np.zeros((4, 2))

        if self.coord_in_obsatcle(x, y):
            # finds rectangle curves
            for i, cor in enumerate(corners):
                cor_next = corners[(i + 1) % len(corners)]
                curves[i] = calc_linear_eq(self.coordinates[cor], self.coordinates[cor_next])

            # calculates distance to every curve
            for curve in curves:
                    distances = [self._calc_distance_to_curve(curve, x, y)]

            # index of closest curve     
            idx = np.argmin(distances)
            return self._calc_coor(curves[idx][0], curves[idx][1], x, y, EXIT_DIST)

        return 0,0    

    # calculates coordinates in EXIT_DISTANCE from given curve 
    def _calc_coor(self, m, b, x, y, distance):
        sgn = np.sign(y - m *x - b)

        # x,y is below the closest curve
        if sgn < 0:
            return self._calc_coor_above(self, m, b, x, y, distance)

        # x,y is above the closest curve
        if sgn > 0:
            return self._calc_coor_below(self, m, b, x, y, distance)

        return 0,0   

    # returns the coor above given line in a given distance
    def _calc_coor_above(self, m, b, x, y, distance):
        c = y - ( -1/m) * x
        x_coor = (c - b - distance * np.sqrt(m**2 + 1)) / (m + 1/m)
        y_coor = (-1/m) * x_coor + c

        return x_coor, y_coor

    # return the coor below the given line in a given distance
    def _calc_coor_below(self, m, b, x, y, distance):
        c = y - ( -1/m) * x
        x_coor = (c - b + distance * np.sqrt(m**2 + 1)) / (m + 1/m)
        y_coor = (-1/m) * x_coor + c 

        return x_coor, y_coor
        
    def _calc_distance_to_curve(self, curve, x, y):
        return abs(curve[0] * x - y + curve[1]) / np.sqrt(curve[0] ** 2 + 1)

    @staticmethod
    def _get_shortest_path(reachable_distances):
        reachable_distances_graph = csr_matrix(reachable_distances)
        dist_matrix, predecessors, sources = dijkstra(csgraph=reachable_distances_graph, directed=True, indices=0,
                                                      return_predecessors=True)
        target = len(predecessors) - 1
        vertex = target
        path = []
        while vertex != 0:
            path.insert(vertex, 0)
            vertex = predecessors[vertex]
            if 0 > vertex or vertex > target:
                return []
        return path

    def _continue_line_to_distance(self, x1, y1, x2, y2, r):
        a = (y1 - y2) / (x1 - x2)
        b = y1 - a*x1
        s = 0 if x2 > x2 else 1
        x = self._solve_quadratic(a**2 + 1, 2*(a*b - x1 - a*y1), x1**2+b**2-2*b*y1-r**2, s)
        y = a*x + b
        return x, y

    @staticmethod
    def _solve_quadratic(a, b, c, s):
        return (-b + np.sqrt(b ** 2 - 4 * a * c) * ((-1) ** s)) / (2 * a)
