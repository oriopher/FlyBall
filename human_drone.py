import cv2

from recognizable_object import RecognizableObject
from drone import Drone
from utils.config_utils import *
from utils.consts import *
from borders import Borders
from camera import Camera
from utils.image_utils import display_frames


def interactive_loop(borders, left_cam, balloon, drone_1):
    """
    The interactive loop for receiving commands from the user.
    :param borders: the Borders.
    :param left_cam: the left Camera.
    :param balloon: the balloon's RecognizableObject.
    :param drone_1: the Drone.
    :return: whether to continue or not.
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

    # the 's' button is set as the detect color of drone_1 in the left_cam cam
    elif key == ord('s'):
        drone_1.detect_color(True)
        print(str_colors_changed)

    # the 'f' button is set as the detect color of drone_1 in the right_cam cam
    elif key == ord('f'):
        drone_1.detect_color(False)
        print(str_colors_changed)

    elif key == ord('t'):
        drone_1.tookoff = True

    elif key == ord('y'):
        drone_1.start_track()

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        return False

    # the 'p' button is set as the save text_color to file
    elif key == ord('p'):
        save_colors(COLORS_FILENAME, [balloon, drone_1.recognizable_object])

    # the 'k' button is set as the read text_color from file
    elif key == ord('k'):
        load_colors(COLORS_FILENAME, [balloon, drone_1.recognizable_object])

    # the 'j' button is set as the saving the borders. can save 4 coordinates
    elif key == ord('j'):
        borders.set_corner(balloon, left_cam)
        print("Saved the {:.0f} point: ({:.0f},{:.0f})".format(borders.index, balloon.x, balloon.y))
        if borders.index == 4:
            borders.save_borders(BORDERS_FILENAME)
            drone_1.set_home((borders.x_middle_1, borders.y_middle))

    # the 'r' button is set as the read text_color from file
    elif key == ord('r'):
        borders.load_borders(BORDERS_FILENAME, left_cam)
        drone_1.set_home((borders.x_middle_1, borders.y_middle))

    elif key == ord('z'):
        drone_1.game_start = 1

    return True


def game_loop(drone_1: Drone, balloon: RecognizableObject, cameras_distance, left: Camera, right: Camera):
    """
    Runs the loop of the game - capturing frames from the video cameras and controlling the drone accordingly.
    :param drone_1: the Drone.
    :param balloon: the RecognizableObject of the balloon.
    :param cameras_distance: the distance between the cameras.
    :param left: the Camera of the left camera.
    :param right: the Camera of the right camera.
    """

    continue_loop = True

    borders = Borders()
    drones = [drone_1]
    recognizable_objects = [balloon, drone_1.recognizable_object]
    load_colors(COLORS_FILENAME, recognizable_objects)
    borders.load_borders(BORDERS_FILENAME, left)

    drone_1.set_home((100, 320))

    drone_1.active = True
    
    while continue_loop:
        state = drone_1.state
        # Capture the video frame by frame
        if not left.capture():
            continue
        if not right.capture():
            continue

        for recognizable_object in recognizable_objects:
            # Process frames
            recognizable_object.detect_and_set_coordinates(left, right, cameras_distance)
            
        left_img, right_img, xy_display = display_frames(balloon, drones, left, right, borders)
        cv2.imshow("Left", left_img)
        cv2.imshow("Right", right_img)
        cv2.imshow("XY Display", xy_display)

        # State2Drones Machine
        state.run(drone_1, balloon, borders)
        transition = state.to_transition(drone_1, balloon, borders)
        if transition:
            state.cleanup(transition, drone_1, balloon, borders)
            state = drone_1.state = state.next(transition)
            print(state)
            state.setup(drone_1, balloon, borders)

        continue_loop = interactive_loop(borders, left, balloon, drone_1)
    
    if drone_1.tookoff:
        drone_1.land()

    # After the loop release the cap object
    left.release()
    right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


def main():
    """
    A game of 1 drone hitting a balloon a human throws to it.
    """
    left_cam = C920_NIR_1
    right_cam = C920_NIR_2

    drone_1 = Drone(1, (0, 191, 255), iface_ip="192.168.10.2")
    balloon = RecognizableObject((255, 54, 89), "balloon")

    distance = 111.9
    game_loop(drone_1, balloon, distance, left_cam, right_cam)


if __name__ == "__main__":
    main()
