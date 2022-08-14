from recognizable.recognizable_object import RecognizableObject
from utils.common import *
from quadrangles.borders import Borders
from images.camera import Camera
from prediction import NumericBallPredictor
from datetime import datetime

COLORS_FILENAME = "pred_colors.txt"


def display_frames_pred(balloon, left_cam, right_cam, borders, pred_coords):
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


def interactive_loop(borders: Borders, left_cam: Camera, balloon: RecognizableObject, start_test) -> bool:
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


def capture_video(balloon: RecognizableObject, cameras_distance, left: Camera, right: Camera):

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

        if borders.in_borders(balloon) and start_test[0] == 1:
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
    left_cam = C920_NIR_1
    right_cam = C920_NIR_2

    balloon = RecognizableObject((255, 54, 89), "balloon")

    distance = 111.9
    capture_video(balloon, distance, left_cam, right_cam)


if __name__ == "__main__":
    main()
