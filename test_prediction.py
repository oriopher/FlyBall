import numpy as np
import cv2

from recognizable_object import RecognizableObject
from utils.image_utils import image_to_show, image_with_circle
from utils.config_utils import save_colors, load_colors
from xy_display import XYDisplay
from utils.consts import *
from borders import Borders
from prediction import NumericBallPredictor
from datetime import datetime


def display_frames_pred(balloon, left_cam, right_cam, borders, pred_coords):
    """
    Displays the frame for predictions.
    :param balloon: the balloon's RecognizableObject.
    :param left_cam: the left camera's Camera.
    :param right_cam: the right camera's Camera.
    :param borders: the Borders.
    :param pred_coords: the coordinates of the prediction.
    """
    recognizable_objects = [balloon]
    texts_coor = ["c({:.0f},{:.0f},{:.0f})".format(balloon.x, balloon.y, balloon.z)]
    texts_vel = [
        "v({:.0f},{:.0f},{:.0f})".format(recognizable_object.vx, recognizable_object.vy, recognizable_object.vz)
        for recognizable_object in recognizable_objects]

    left_img = image_to_show(left_cam.last_capture.image,
                             [recognizable_object.frame_left for recognizable_object in recognizable_objects],
                             True, texts_coor, (150, 250, 200))

    left_img = borders.draw_borders(left_img, recognizable_objects, color_in=(0, 240, 0), color_out=(0, 0, 240))
    if np.any(pred_coords):
        left_img = image_with_circle(left_cam, left_img, pred_coords, rad_phys=10)
    cv2.imshow("left_cam", left_img)

    right_img = image_to_show(right_cam.last_capture.image,
                              [recognizable_object.frame_right for recognizable_object in recognizable_objects], True,
                              texts_vel, (240, 150, 240))
    cv2.imshow("right_cam", right_img)
    
    xy_display = XYDisplay.get_xy_display(borders, balloon, [])
    cv2.imshow('XY Display', xy_display)


def interactive_loop(borders, left_cam, balloon, start_test):
    """
    The interactive loop for receiving commands from the user.
    :param borders: the Borders.
    :param left_cam: the left camera's Camera.
    :param balloon: the balloon's RecognizableObject.
    :param start_test: whether the test is started or not.
    :return: whether to continue the loop or not.
    """
    key = cv2.waitKey(1) & 0xFF
    str_colors_changed = "Color bounds changed"

    # the 'v' button is set as the detect color of recognizable_object in the left_cam cam
    if key == ord('v'):
        balloon.detect_color(True)
        print(str_colors_changed)

    # the 'n' button is set as the detect color of recognizable_object in the right_cam cam
    elif key == ord('n'):
        balloon.detect_color(False)
        print(str_colors_changed)

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        return False

    # the 'p' button is set as the save text_colors to file
    elif key == ord('p'):
        save_colors(COLORS_FILENAME, [balloon])

    # the 'k' button is set as the read text_colors from file
    elif key == ord('k'):
        load_colors(COLORS_FILENAME, [balloon])

    # the 'j' button is set as the saving the borders. can save 4 coordinates
    elif key == ord('j'):
        borders.set_corner(balloon, left_cam)
        print("Saved the %.0f coordinate: (%.0f,%.0f)" % (borders.index, balloon.x, balloon.y))
        if borders.index == 4:
            borders.save_borders(BORDERS_FILENAME)

    # the 'r' button is set as the read text_colors from file
    elif key == ord('r'):
        borders.load_borders(BORDERS_FILENAME, left_cam)

    elif key == ord('z'):
        start_test[0] = 1
    
    elif key == ord('x'):
        start_test[0] = 0

    return True


def test_prediction_loop(balloon, cameras_distance, left, right):
    """
    Runs the loop of the test - capturing frames from the video cameras and presenting predictions when wanted.
    :param balloon: the RecognizableObject of the balloon.
    :param cameras_distance: the distance between the cameras.
    :param left: the Camera of the left camera.
    :param right: the Camera of the right camera.
    """

    continue_loop = True
    pred = None
    start_pred_timer = None
    start_test = [0]

    borders = Borders()
    recognizable_objects = [balloon]
    load_colors(COLORS_FILENAME, recognizable_objects)
    borders.load_borders(BORDERS_FILENAME, left)
    
    while continue_loop:
        # Capture the video frame by frame
        if not left.capture():
            continue
        if not right.capture():
            continue

        for recognizable_object in recognizable_objects:
            # Process frames
            recognizable_object.detect_and_set_coordinates(left, right, cameras_distance)

        if pred:
            pred_coords = pred.get_prediction((datetime.now() - start_pred_timer).total_seconds())
        else:
            pred_coords = None
            
        display_frames_pred(balloon, left, right, borders, pred_coords)

        if borders.is_set and borders.in_borders(balloon) and start_test[0] == 1:
            pred = NumericBallPredictor(balloon)
            start_pred_timer = datetime.now()
            start_test[0] = 2

        if start_test[0] == 0:
            pred = None
            start_pred_timer = None

        continue_loop = interactive_loop(borders, left, balloon, start_test)

    # After the loop release the cap object
    left.release()
    right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


def main():
    """
    A test for the prediction's of the balloon location, presenting the prediction wen requested.
    """
    left_cam = C920_NIR_1
    right_cam = C920_NIR_2

    balloon = RecognizableObject((255, 54, 89), "balloon")

    distance = 111.9
    test_prediction_loop(balloon, distance, left_cam, right_cam)


if __name__ == "__main__":
    main()
