import numpy as np
import cv2

from frame import Frame


class Camera:

    def __init__(self, fov_horz: float, fov_vert: float, index: int, is_flipped=False) -> None:
        self.fov_horz = np.radians(fov_horz)
        self.fov_vert = np.radians(fov_vert) 
        self.index = index
        self.flip = -1 if is_flipped else 1
        self.is_flipped = is_flipped
        self.vid = None
        self.last_capture = None

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
