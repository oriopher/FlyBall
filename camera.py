import numpy as np
import cv2
from time import time


class Camera:

    def __init__(self, fov: float, index: int, is_flipped=False) -> None:
        self.fov = np.radians(fov)
        self.index = index
        self.flip = -1 if is_flipped else 1
        self.is_flipped = is_flipped
        self.vid = cv2.VideoCapture(index)
        self._calc_fps()

    def _calc_fps(self) -> None:
        NUM_SECS = 2
        frame_counter = 0
        t0 = time()
        while time() - t0 < NUM_SECS:
            self.vid.read()
            frame_counter += 1
        self.fps = frame_counter // NUM_SECS
