from recognizable_object import RecognizableObject
from loop_status import Status
from drone import Drone
from common import *


def interactive_loop(borders: Borders, loop_status: Status, left_cam: Camera, balloon: RecognizableObject, drone: Drone) -> bool:
    key = cv2.waitKey(1) & 0xFF
    str_colors_changed = "Color bounds changed"

    # the 'v' button is set as the detect color of recognizable_object in the left cam
    if key == ord('v'):
        balloon.frame_left.detect_color()
        print(str_colors_changed)

    # the 'n' button is set as the detect color of recognizable_object in the right cam
    elif key == ord('n'):
        balloon.frame_right.detect_color()
        print(str_colors_changed)

    # the 's' button is set as the detect color of drone in the left cam
    elif key == ord('s'):
        drone.frame_left.detect_color()
        print(str_colors_changed)

    # the 'f' button is set as the detect color of drone in the right cam
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

    # the 'r' button is set as the read text_colors from file
    elif key == ord('r'):
        borders.read_borders(BORDERS_FILENAME)

    # the 'a' button is set to abort hitting state back to seek middle
    elif key == ord('a'):
        drone.stop_hit()

    elif key == ord('z'):
        loop_status.testing = 1

    return True


def capture_video(drone: Drone, balloon: RecognizableObject, cameras_distance, left: Camera, right: Camera, method='parallel'):
    vid_left = cv2.VideoCapture(left.index)
    vid_right = cv2.VideoCapture(right.index)

    frame_counter = 0
    continue_loop = True

    loop_status = Status()
    borders = Borders()
    read_colors(COLORS_FILENAME, [balloon, drone])
    borders.read_borders(BORDERS_FILENAME)
    recognizable_objects = [balloon, drone]

    while continue_loop:
        state = drone.state
        frame_counter = frame_counter+1
        # Capture the video frame by frame
        ret_left, image_left = vid_left.read()
        if not ret_left:
            continue
        real_image_left = image_left
        ret_right, image_right = vid_right.read()
        if not ret_right:
            continue
        for recognizable_object in recognizable_objects:
            recognizable_object.set_frames(real_image_left, image_right)

            # Process frames
            recognizable_object.detect_and_set_coordinates(left, right, cameras_distance, frame_counter)

        display_frames(recognizable_objects, real_image_left, image_right, left, borders, loop_status)

        # State Machine
        state_kwargs = {'drone': drone, 'balloon': balloon, 'loop_status': loop_status, 'borders': borders}
        state.run(**state_kwargs)
        transition = state.to_transition(**state_kwargs)
        if transition:
            state.cleanup(transition, **state_kwargs)
            state = loop_status.state = state.next(transition)
            state.setup(**state_kwargs)

        continue_loop = interactive_loop(borders, loop_status, left, balloon, drone)
    
    if drone.tookoff:
        drone.land()

    # After the loop release the cap object
    vid_left.release()
    vid_right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


if __name__ == "__main__":
    continue_test = True

    left = MAYA_PHONE_NIR
    right = EFRAT_PHONE_NIR

    distance = 69
    capture_video(Drone(1,  (0, 191, 255), iface_ip="192.168.0.1"), RecognizableObject((255, 54, 89)), distance, left, right, method='parallel')
