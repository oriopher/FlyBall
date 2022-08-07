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
    EXIT_MARGIN = 100
    MARGINS = 25
    MARGINS_END = 25
    MARGINS_START = 5
    MARGINS_SIDES = 10

    def __init__(self, drone, left_cam):
        self.start = (drone.x, drone.y)
        self.end = (drone.dest_coords[0], drone.dest_coords[1])
        self.quad = Quadrangle(self.calc_corners(), left_cam)

    @property
    def coordinates(self):
        return self.quad.coordinates

    def _calc_middles(self):
        x_middle = (self.start[0] + self.end[0]) / 2
        y_middle = (self.start[1] + self.end[1]) / 2

        # rectangle parallel to y axis
        if self.end[0] == self.start[0]:
            x_middle_low = x_middle - self.MARGINS_SIDES
            x_middle_upper = x_middle + self.MARGINS_SIDES
            y_middle_low = y_middle
            y_middle_upper = y_middle

        # rectangle parallel to x axis
        elif self.end[1] == self.start[1]:
            x_middle_low = x_middle
            x_middle_upper = x_middle
            y_middle_low = y_middle - self.MARGINS_SIDES
            y_middle_upper = y_middle + self.MARGINS_SIDES

        else:
            m_track = (self.start[1] - self.end[1]) / (self.start[0] - self.end[0])
            b_track = y_middle - m_track * x_middle

            x_middle_low, y_middle_low = self._calc_coor_below(m_track, b_track, x_middle, y_middle, self.MARGINS_SIDES)
            x_middle_upper, y_middle_upper = self._calc_coor_above(m_track, b_track, x_middle, y_middle, self.MARGINS_SIDES)

        return x_middle_low, y_middle_low, x_middle_upper, y_middle_upper

    # finds the vertexes of the rectangle
    def calc_corners(self):
        x_middle_low, y_middle_low, x_middle_upper, y_middle_upper = self._calc_middles()
        length_end = 0.5 * np.sqrt((self.start[0] - self.end[0]) ** 2 + (self.start[1] - self.end[1]) ** 2) + self.MARGINS_END
        length_start = 0.5 * np.sqrt((self.start[0] - self.end[0]) ** 2 + (self.start[1] - self.end[1]) ** 2) + self.MARGINS_START
                
        # Horizontal rectangle
        if x_middle_upper == x_middle_low:
            if self.end[0] >= self.start[0]:
                a_x = x_middle_upper - length_start
                a_y = y_middle_upper
                
                d_x = x_middle_upper + length_end
                d_y = y_middle_upper
                
                b_x = x_middle_low - length_start
                b_y = y_middle_low

                c_x = x_middle_low + length_end
                c_y = y_middle_low
        
            else:
                a_x = x_middle_upper - length_end
                a_y = y_middle_upper
                
                d_x = x_middle_upper + length_start
                d_y = y_middle_upper
                
                b_x = x_middle_low - length_end
                b_y = y_middle_low

                c_x = x_middle_low + length_start
                c_y = y_middle_low

            
        # Vertical rectangle
        elif y_middle_upper == y_middle_low:
            if self.end[1] >= self.start[1]:
                a_y = y_middle_upper - length_start
                a_x = x_middle_upper

                d_y = y_middle_upper + length_end
                d_x = x_middle_upper

                b_y = y_middle_low - length_start
                b_x = x_middle_low

                c_y = y_middle_low + length_end
                c_x = x_middle_low

            else:
                a_y = y_middle_upper - length_end
                a_x = x_middle_upper

                d_y = y_middle_upper + length_start
                d_x = x_middle_upper

                b_y = y_middle_low - length_end
                b_x = x_middle_low

                c_y = y_middle_low + length_start
                c_x = x_middle_low    

        # Slanted rectangle
        else:
            # Calculate slope of the side
            m = (x_middle_upper - x_middle_low) / (y_middle_low - y_middle_upper)
            
            # Calculate displacements along axes
            dx_end = (length_end / np.sqrt(1 + (m ** 2)))
            dy_end = m * dx_end
            dx_start = (length_start / np.sqrt(1 + (m ** 2)))
            dy_start = m *dx_start
            
            if (m < 0 and self.end[1] > self.start[1]) or (m > 0 and self.end[1] < self.start[1]):
                a_x = x_middle_upper - dx_start
                a_y = y_middle_upper - dy_start

                d_x = x_middle_upper + dx_end
                d_y = y_middle_upper + dy_end

                b_x = x_middle_low - dx_start
                b_y = y_middle_low - dy_start

                c_x = x_middle_low + dx_end
                c_y = y_middle_low + dy_end

            else:
                a_x = x_middle_upper - dx_end
                a_y = y_middle_upper - dy_end

                d_x = x_middle_upper + dx_start
                d_y = y_middle_upper + dy_start

                b_x = x_middle_low - dx_end
                b_y = y_middle_low - dy_end

                c_x = x_middle_low + dx_start
                c_y = y_middle_low + dy_start                    

        # retruns rectangle's coordinates
        return [(c_x, c_y), (b_x, b_y), (d_x, d_y), (a_x, a_y)]

    # checks if the drone is inside the obstacle
    def inside_obstacle(self, drone):
        return self.coord_in_obstacle(drone.x, drone.y)

    # checks if the given coordinate is inside the obstacle
    def coord_in_obstacle(self, x, y):
        return self.quad.coordinate_in_quadrangle(x, y)

    # draws the obstacle on left frame
    def draw_obstacle(self, show_img, recognizable_object, color_in=(240, 0, 0), color_out=(0, 240, 0)):
        color = color_out

        if self.inside_obstacle(recognizable_object):
            color = color_in
        show_img = self.quad.draw_quadrangle(show_img, color)

        return show_img

    def bypass_obstacle_coordinates(self, source, target):

        if self.coord_in_obstacle(source[0], source[1]):
            return self._get_exit_dest(source[0], source[1], self.EXIT_MARGIN)

        if self.coord_in_obstacle(target[0], target[1]):
            new_target = self._get_exit_dest(target[0], target[1])
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
        print(next_point)
        return self._continue_line_to_distance(*source, *next_point, np.linalg.norm(np.array(source) - np.array(target)))

    def _get_reachable_distances(self, point, vertices):
        distances = np.linalg.norm(vertices - point, axis=1)
        reachable = 1 - np.apply_along_axis(lambda point2: self.quad.cross_quadrangle(point, point2), 1, vertices)
        return reachable * distances

    def _get_corners_reachable_distances(self):
        obstacle_distances = distance_matrix(self.coordinates, self.coordinates)
        for i, j in [(0, 3), (1, 2)]:
            obstacle_distances[i][j] = 0
            obstacle_distances[j][i] = 0
        return obstacle_distances

    def _get_preparation_dest(self):
        distances = np.array([])
        # calculates squared distance to every point of te rectangle
        for coordinate in self.coordinates:
            dist_start = (self.start[0] - coordinate[0]) ** 2 + (self.start[1] - coordinate[1]) ** 2
            dist_end = (self.end[0] - coordinate[0]) ** 2 + (self.end[1] - coordinate[1]) ** 2
            distances = np.append(distances, dist_end - dist_start)

        # finds 2 nearest points to end point
        first_point = self.coordinates[np.argmin(distances)]
        distances[np.argmin(distances)] = distances[np.max(distances)] + 1
        second_point = self.coordinates[np.argmin(distances)]

        x_dest = first_point[0] + second_point[0] / 2
        y_dest = first_point[1] + second_point[1] / 2

        return x_dest, y_dest

    # returns the nearest exit point from obstacle
    def _get_exit_dest(self, x, y, exit_dist=MARGINS):
        curves = np.zeros((4, 2))

        if self.coord_in_obstacle(x, y):
            # finds rectangle curves
            for i, cor in enumerate(self.quad.CORNERS):
                cor_next = self.quad.CORNERS[(i + 1) % len(self.quad.CORNERS)]
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

    def _continue_line_to_distance(self, x1, y1, x2, y2, r):
        if x1 == x2:
            s = 0 if y2 > y1 else 1
            return x1, y1 + r * (-1)**s
        a = (y1 - y2) / (x1 - x2)
        b = y1 - a*x1
        s = 0 if x2 > x2 else 1
        x = self._solve_quadratic(a**2 + 1, 2*(a*b - x1 - a*y1), x1**2+b**2-2*b*y1-r**2, s)
        y = a*x + b
        return x, y

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
        return (-b + np.sqrt(b ** 2 - 4 * a * c) * ((-1) ** s)) / (2 * a)
