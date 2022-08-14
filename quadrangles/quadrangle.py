import numpy as np
import cv2
from scipy.spatial import distance as dist

from utils.common import phys_to_left_pix
from utils.consts import FLOOR_HEIGHT
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

    # The order of the corners.
    CORNERS = [0, 1, 3, 2]

    def __init__(self, coordinates, left_cam):
        """
        Initializes a quadrangle.
        :param coordinates: a list (or numpy array) of the coordinates of the corners (in any order).
        :param left_cam: the Camera object of the left camera.
        """
        self.coordinates = self._order_points(np.array(coordinates))
        self._arrangement = self._generate_arrangement()
        self._point_location = Arr_point_location(self._arrangement)
        self._pixels_coordinates = np.zeros((4, 2), dtype=int)
        if left_cam:
            self._calc_corners_pix(left_cam)

    def __str__(self):
        """
        :return: string representation of the quadrangle.
        """
        return "".join([self._coordinate_str(coord) for coord in self.coordinates])

    @staticmethod
    def _coordinate_str(point):
        """
        :param point: a coordinate (of a corner).
        :return: a string representation of a single coordinate.
        """
        return "{:.2f},{:.2f}\n".format(*point)

    def point_in_quadrangle(self, x, y):
        """
        Checks if a point is inside the quadrangle (the corners and sides are outside).
        :param x: the x coordinate of the point.
        :param y: the y coordinate of the point.
        :return: True if (x,y) is inside the quadrangle and False otherwise.
        """
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
        """
        Checks if an inputted section crosses the quadrangle (if it crosses only  a corner it does not).
        :param point_a: first end of the section.
        :param point_b: second end of the section.
        :return: True if the section connecting point_a and point_b crosses the quadrangle.
        """
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
        """
        Draws the quadrangle on given image from the left camera.
        :param show_img: the frame of the left camera.
        :param color: the color in which to use for the quadrangles sides.
        :return: show_image with the quadrangle on it.
        """
        for i, cor in enumerate(self.CORNERS):
            cor_next = self.CORNERS[(i + 1) % len(self.CORNERS)]
            show_img = cv2.line(show_img, (self._pixels_coordinates[cor][0], self._pixels_coordinates[cor][1]),
                                (self._pixels_coordinates[cor_next][0], self._pixels_coordinates[cor_next][1]), color,
                                thickness=2)

        return show_img

    def _calc_corners_pix(self, left_cam):
        """
        Calculates and saves the location of the corners of the quadrangle in pixels (of the left camera).
        :param left_cam: the left cameras Camera object.
        """
        # calculate the coordinates pixels location on frame
        if left_cam.last_capture is None:
            return
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
        """
        Generates a CGALPY arrangement of the quadrangle.
        """
        # print("obstacle: ", self.coordinates)
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
        """
        :param point: a 2d point (tuple/list/numpy array).
        :return: a CGALPY point with the same coordinates of the given point.
        """
        return Point2(*[FT(float(coord)) for coord in point])

    @staticmethod
    def _order_points(coordinates):
        """
        order the inputted point in the shape presented at the classes docstring
        code copied from: https://pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/
        :param coordinates: the coordinates of the quadrangles corners
        :return: the coordinates of the quadrangles corners in the correct order
        """
        # sort the points based on their x-coordinates
        xSorted = coordinates[np.argsort(coordinates[:, 0]), :]
        # grab the left-most and right-most points from the sorted
        # x-point points
        leftMost = xSorted[:2, :]
        rightMost = xSorted[2:, :]
        # now, sort the left-most coordinates according to their
        # y-coordinates so we can grab the top-left and bottom-left
        # points, respectively
        leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
        bl, tl = leftMost
        # now that we have the top-left point, use it as an
        # anchor to calculate the Euclidean distance between the
        # top-left and right-most points; by the Pythagorean
        # theorem, the point with the largest distance will be
        # our bottom-right point
        D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
        br, tr = rightMost[np.argsort(D)[::-1], :]
        # return the coordinates in top-left, top-right,
        # bottom-right, and bottom-left order
        return np.array([br, bl, tr, tl])
