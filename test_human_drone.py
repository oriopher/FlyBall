import cv2
from recognizable_object import RecognizableObject
from loop_status import Status
from drone import Drone
from common import read_colors, write_colors, display_frames, BORDERS_FILENAME, COLORS_FILENAME, EFRAT_WEB, NIR_PHONE_NIR
from borders import Borders
from camera import Camera


def interactive_loop(borders: Borders, loop_status: Status, left_cam: Camera, balloon: RecognizableObject, drone: Drone) -> bool:
    key = cv2.waitKey(1) & 0xFF
    str_colors_changed = "Color bounds changed"

    # the 'v' button is set as the detect color of recognizable_object in the left_cam cam
    if key == ord('v'):
        balloon.frame_left.detect_color()
        print(str_colors_changed)

    # the 'n' button is set as the detect color of recognizable_object in the right_cam cam
    elif key == ord('n'):
        balloon.frame_right.detect_color()
        print(str_colors_changed)

    # the 's' button is set as the detect color of drone in the left_cam cam
    elif key == ord('s'):
        drone.frame_left.detect_color()
        print(str_colors_changed)

    # the 'f' button is set as the detect color of drone in the right_cam cam
    elif key == ord('f'):
        drone.frame_right.detect_color()
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
        write_colors(COLORS_FILENAME, [balloon, drone])

    # the 'k' button is set as the read text_colors from file
    elif key == ord('k'):
        read_colors(COLORS_FILENAME, [balloon, drone])

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

    # the 'a' button is set to abort hitting state back to seek middle
    elif key == ord('a'):
        drone.stop_hit()

    elif key == ord('z'):
        loop_status.testing = 1

    return True


def capture_video(drone: Drone, balloon: RecognizableObject, cameras_distance, left: Camera, right: Camera):

    continue_loop = True

    loop_status = Status()
    borders = Borders()
    recognizable_objects = [balloon, drone]
    read_colors(COLORS_FILENAME, recognizable_objects)
    borders.read_borders(BORDERS_FILENAME)
    if borders.set_borders:
        drone.set_middle((borders.x_middle, borders.y_middle))

    while continue_loop:
        state = drone.state
        print(state)
        # Capture the video frame by frame
        if not left.capture():
            continue
        if not right.capture():
            continue

        for recognizable_object in recognizable_objects:
            # Process frames
            recognizable_object.detect_and_set_coordinates(left, right, cameras_distance)
            
        display_frames(recognizable_objects, left, right, borders, loop_status)

        # State Machine
        state_kwargs = {'drone': drone, 'balloon': balloon, 'loop_status': loop_status, 'borders': borders}
        state.run(**state_kwargs)
        transition = state.to_transition(**state_kwargs)
        if transition:
            state.cleanup(transition, **state_kwargs)
            state = drone.state = state.next(transition)
            state.setup(**state_kwargs)

        continue_loop = interactive_loop(borders, loop_status, left, balloon, drone)
    
    if drone.tookoff:
        drone.land()

    # After the loop release the cap object
    left.release()
    right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


def main():
    left_cam = NIR_PHONE_NIR
    right_cam = EFRAT_WEB

    distance = 74
    capture_video(Drone(1, (0, 191, 255), 7, iface_ip="192.168.10.2"), RecognizableObject((255, 54, 89), 11.3, "balloon"), distance,
                  left_cam, right_cam)


if __name__ == "__main__":
    main()
