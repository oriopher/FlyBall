import numpy as np
import cv2

from frame import Frame


class Camera:

    def __init__(self, fov_horz, fov_vert, index, is_flipped=False):
        self.fov_horz = np.radians(fov_horz)
        self.fov_vert = np.radians(fov_vert) 
        self.index = index
        self.flip = -1 if is_flipped else 1
        self.is_flipped = is_flipped
        self.vid = None
        self.last_capture = None

    def __str__(self):
        return "%.3f,%.3f,%.0f,%.0f\n" % (self.fov_horz, self.fov_vert, self.last_capture.x_n_pix, self.last_capture.z_n_pix)

    def capture(self):
        if not self.vid:
            self.vid = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        ret, image = self.vid.read()
        if not ret:
            return False
        self.last_capture = Frame(image)

        return True

    def release(self):
        if self.vid:
            self.vid.release()
