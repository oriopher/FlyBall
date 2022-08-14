import numpy as np
import cv2

from frame import Frame


class Camera:

    def __init__(self, fov_horz, fov_vert, index, is_flipped=False):
        """
        Initializes the camera.
        :param fov_horz: the cameras horizontal field of view angle.
        :param fov_vert: the cameras vertical field of view angle.
        :param index: the index of the camera on the computer.
        :param is_flipped: is the feed of the camera flipped.
        """
        self.fov_horz = np.radians(fov_horz)
        self.fov_vert = np.radians(fov_vert) 
        self.index = index
        self.flip = -1 if is_flipped else 1
        self.is_flipped = is_flipped
        self.vid = None
        self.last_capture = None

    def __str__(self):
        """
        :return: a string representation of the camera.
        """
        fov_str = "{:.3f},{:.3f}".format(self.fov_horz, self.fov_vert)
        n_pix_str = "{:.0f},{:.0f}".format(self.last_capture.x_n_pix, self.last_capture.z_n_pix) if self.last_capture else ""
        return fov_str + n_pix_str + '\n'

    def capture(self):
        """
        Capture an image from the camera and save it as a Frame. if the video capture is not yet set - initialize it.
        :return: whether the capture succeeded or not.
        """
        if not self.vid:
            self.vid = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        ret, image = self.vid.read()
        if not ret:
            return False
        self.last_capture = Frame(image)

        return True

    def release(self):
        """
        Release the video capture of the camera.
        """
        if self.vid:
            self.vid.release()
