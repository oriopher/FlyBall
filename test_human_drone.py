import cv2
from recognizable_object import RecognizableObject
from drone import Drone
from common import *
from borders import Borders
from camera import Camera


def interactive_loop(borders: Borders, left_cam: Camera, balloon: RecognizableObject, drone: Drone) -> bool:
    key = cv2.waitKey(1) & 0xFF
    str_colors_changed = "Color bounds changed"

    recognizable_objects = [balloon, drone.recognizable_object]
    # the 'v' button is set as the detect color of recognizable_object in the left_cam cam
    if key == ord('v'):
        balloon.detect_color(True)
        print(str_colors_changed)

    # the 'n' button is set as the detect color of recognizable_object in the right_cam cam
    elif key == ord('n'):
        balloon.detect_color(False)
        print(str_colors_changed)

    # the 's' button is set as the detect color of drone in the left_cam cam
    elif key == ord('s'):
        drone.detect_color(True)
        print(str_colors_changed)

    # the 'f' button is set as the detect color of drone in the right_cam cam
    elif key == ord('f'):
        drone.detect_color(False)
        print(str_colors_changed)

    elif key == ord('t'):
        drone.takeoff()

    elif key == ord('y'):
        drone.start_track()

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        return False

    # the 'p' button is set as the save text_colors to file
    elif key == ord('p'):
        write_colors(COLORS_FILENAME, recognizable_objects)

    # the 'k' button is set as the read text_colors from file
    elif key == ord('k'):
        read_colors(COLORS_FILENAME, recognizable_objects)

    # the 'j' button is set as the saving the borders. can save 4 coordinates
    elif key == ord('j'):
        borders.set_image(balloon, left_cam)
        print("Saved the %.0f coordinate: (%.0f,%.0f)" % (borders.index, balloon.x, balloon.y))
        if borders.index == 4:
            borders.write_borders(BORDERS_FILENAME)
            drone.set_middle((borders.x_middle, borders.y_middle))

    # the 'r' button is set as the read text_colors from file
    elif key == ord('r'):
        borders.read_borders(BORDERS_FILENAME)
        drone.set_middle((borders.x_middle, borders.y_middle))

    elif key == ord('z'):
        drone.testing = 1

    return True


def capture_video(drone: Drone, balloon: RecognizableObject, cameras_distance, left: Camera, right: Camera):

    continue_loop = True

    borders = Borders()
    recognizable_objects = [balloon, drone.recognizable_object]
    read_colors(COLORS_FILENAME, recognizable_objects)
    borders.read_borders(BORDERS_FILENAME)
    if borders.set_borders:
        drone.set_middle((borders.x_middle, borders.y_middle))

    while continue_loop:
        state = drone.state
        # Capture the video frame by frame
        if not left.capture():
            continue
        if not right.capture():
            continue

        for recognizable_object in recognizable_objects:
            # Process frames
            recognizable_object.detect_and_set_coordinates(left, right, cameras_distance)
            
        display_frames(balloon, drone, left, right, borders)

        # State Machine
        state.run(drone, balloon, borders)
        transition = state.to_transition(drone, balloon, borders)
        if transition:
            state.cleanup(transition, drone, balloon, borders)
            state = drone.state = state.next(transition)
            print(state)
            state.setup(drone, balloon, borders)

        continue_loop = interactive_loop(borders, left, balloon, drone)
    
    if drone.tookoff:
        drone.land()

    # After the loop release the cap object
    left.release()
    right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


def main():
    left_cam = C920_NIR_2
    right_cam = C920_NIR_1

    distance = 111.9
    capture_video(Drone(1, (0, 191, 255), 7, iface_ip="192.168.10.2"), RecognizableObject((255, 54, 89), 11.3, "balloon"), distance,
                  left_cam, right_cam)


if __name__ == "__main__":
    main()
