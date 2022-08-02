import numpy as np
import datetime
import cv2


class Camera:

    def __init__(self, fov_horz: float, fov_vert: float, index: int, is_flipped=False) -> None:
        self.fov_horz = np.radians(fov_horz)
        self.fov_vert = np.radians(fov_vert) 
        self.index = index
        self.flip = -1 if is_flipped else 1
        self.is_flipped = is_flipped
        self.vid = None
        self.last_capture = None
        self.last_capture_time = 0

    def capture(self):
        if not self.vid:
            self.vid = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        ret, image = self.vid.read()
        if not ret:
            return False
        self.last_capture = image
        self.last_capture_time = datetime.datetime.now()
        return True

    def release(self):
        if self.vid:
            self.vid.release()
