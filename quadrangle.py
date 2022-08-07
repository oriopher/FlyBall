import numpy as np
import cv2
from scipy.spatial import distance as dist

from common import phys_to_left_pix, FLOOR_HEIGHT
import CGALPY

Ker = CGALPY.Ker
FT = Ker.FT
Point2 = Ker.Point_2
Segment_2 = Ker.Segment_2

Aos2 = CGALPY.Aos2
Arrangement_2 = Aos2.Arrangement_2
TPoint = Aos2.Traits.Point_2
Curve_2 = Aos2.Traits.X_monotone_curve_2

Vertex = Arrangement_2.Vertex
Halfedge = Arrangement_2.Halfedge
Face = Arrangement_2.Face

Arr_point_location = Aos2.Arr_trapezoid_ric_point_location


class Quadrangle:
    """
    Quadrangles corners are labeled as follows (any order of inputted coordinates will be arranged in this manner):
    (0 and 2 are first set to be the 2 rightmost points and then their indices is set so 0 is the lower between them, and so on)
    ###############################
    #### 3-------------------2 ####
    #### |-------------------| ####
    #### |-------------------| ####
    #### |-------------------| ####
    #### |-------------------| ####
    #### 1-------------------0 ####
    ###############################
    """

    CORNERS = [0, 1, 3, 2]

    def __init__(self, coordinates, left_cam):
        self.coordinates = self._order_points(np.array(coordinates))
        self._arrangement = self._generate_arrangement()
        self._point_location = Arr_point_location(self._arrangement)
        self._pixels_coordinates = np.zeros((4, 2), dtype=int)
        if left_cam:
            self._calc_edges_pix(left_cam)

    def __str__(self):
        return "".join([self._coordinate_str(coord) for coord in self.coordinates])

    @staticmethod
    def _coordinate_str(coordinate):
        return "%.2f,%.2f\n" % (coordinate[0], coordinate[1])

    def coordinate_in_quadrangle(self, x, y):
        point = self._point_to_cgal((x, y))
        query = TPoint(point.x(), point.y())
        obj = self._point_location.locate(query)
        v = Vertex()
        he = Halfedge()
        f = Face()
        if obj.get_vertex(v):
            return False
        elif obj.get_halfedge(he):
            return False
        elif obj.get_face(f):
            if f.is_unbounded():
                return False
            else:
                return True

    def cross_quadrangle(self, point_a, point_b):
        res = []
        points = [self._point_to_cgal(point) for point in [point_a, point_b]]
        curve = Curve_2(*points)
        Aos2.zone(self._arrangement, curve, res, self._point_location)

        for obj in res:
            if type(obj) is Vertex:
                continue
            elif type(obj) is Halfedge:
                continue
            elif type(obj) is Face:
                if obj.is_unbounded():
                    continue
                else:
                    return True
        return False

    # draws quadrangle on frame
    def draw_quadrangle(self, show_img, color=(240, 0, 240)):
        for i, cor in enumerate(self.CORNERS):
            cor_next = self.CORNERS[(i + 1) % len(self.CORNERS)]
            show_img = cv2.line(show_img, (self._pixels_coordinates[cor][0], self._pixels_coordinates[cor][1]),
                                (self._pixels_coordinates[cor_next][0], self._pixels_coordinates[cor_next][1]), color,
                                thickness=2)

        return show_img

    def _calc_edges_pix(self, left_cam):
        # calculate the coordinates pixels location on frame
        for i in range(len(self._pixels_coordinates)):
            if self.coordinates[i][1] != 0:
                self._pixels_coordinates[i][0], self._pixels_coordinates[i][1] = phys_to_left_pix(self.coordinates[i][0],
                                                                                                  self.coordinates[i][1],
                                                                                                  FLOOR_HEIGHT - 10,
                                                                                                  left_cam.last_capture.x_n_pix,
                                                                                                  left_cam.last_capture.z_n_pix,
                                                                                                  left_cam.fov_horz,
                                                                                                  left_cam.fov_vert)

    def _generate_arrangement(self):
        points = [self._point_to_cgal(point) for point in self.coordinates]
        curves = []
        for i, cor in enumerate(self.CORNERS):
            cor_next = self.CORNERS[(i + 1) % len(self.CORNERS)]
            curves.append(Curve_2(points[cor], points[cor_next]))
        arr = Arrangement_2()
        Aos2.insert(arr, curves)
        return arr

    @staticmethod
    def _point_to_cgal(point):
        return Point2(*[FT(float(coord)) for coord in point])

    @staticmethod
    def _order_points(coordinates):
        """
        order the inputted coordinate in the shape presented at the classes docstring
        code copied from: https://pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/
        :param coordinates: the coordinates of the quadrangles corners
        :return: the coordinates of the quadrangles corners in the correct order
        """
        # sort the points based on their x-coordinates
        xSorted = coordinates[np.argsort(coordinates[:, 0]), :]
        # grab the left-most and right-most points from the sorted
        # x-coordinate points
        leftMost = xSorted[:2, :]
        rightMost = xSorted[2:, :]
        # now, sort the left-most coordinates according to their
        # y-coordinates so we can grab the top-left and bottom-left
        # points, respectively
        leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
        bl, tl = leftMost
        # now that we have the top-left coordinate, use it as an
        # anchor to calculate the Euclidean distance between the
        # top-left and right-most points; by the Pythagorean
        # theorem, the point with the largest distance will be
        # our bottom-right point
        D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
        br, tr = rightMost[np.argsort(D)[::-1], :]
        # return the coordinates in top-left, top-right,
        # bottom-right, and bottom-left order
        return np.array([br, bl, tr, tl])

