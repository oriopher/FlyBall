import numpy as np

class Camera:

    def __init__(self, fov: float, index: int, is_flipped=False) -> None:
        self.fov = np.radians(fov)
        self.index = index
        self.flip = -1 if is_flipped else 1
        self.is_flipped = is_flipped
