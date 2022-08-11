import cv2, numpy as np
from recognizable_object import RecognizableObject
from drone import Drone
from common import MAYA_PHONE_NIR, ORI_PHONE, load_colors, save_colors, display_frames, BORDERS_FILENAME, COLORS_FILENAME, EFRAT_WEB, NIR_PHONE_NIR
from borders import Borders
from camera import Camera


def interactive_loop(borders: Borders, left_cam: Camera, balloon: RecognizableObject, drone_1: Drone, drone_2: Drone) -> bool:
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

    # the 's' button is set as the detect color of drone_1 in the left_cam cam
    elif key == ord('s'):
        drone_1.frame_left.detect_color()
        print(str_colors_changed)

    # the 'f' button is set as the detect color of drone_1 in the right_cam cam
    elif key == ord('f'):
        drone_1.frame_right.detect_color()
        print(str_colors_changed)

    # the 'w' button is set as the detect color of drone_2 in the right_cam cam
    elif key == ord('w'):
        drone_2.frame_left.detect_color()
        print(str_colors_changed)

    # the 'e' button is set as the detect color of drone_2 in the right_cam cam
    elif key == ord('e'):
        drone_2.frame_right.detect_color()
        print(str_colors_changed)

    elif key == ord('t'):
        drone_1.takeoff()
        drone_2.takeoff()

    elif key == ord('y'):
        drone_1.start_track()
        drone_2.start_track()

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        return False

    # the 'p' button is set as the save text_color to file
    elif key == ord('p'):
        save_colors(COLORS_FILENAME, [balloon, drone_1, drone_2])

    # the 'k' button is set as the read text_color from file
    elif key == ord('k'):
        load_colors(COLORS_FILENAME, [balloon, drone_1, drone_2])

    # the 'j' button is set as the saving the borders. can save 4 coordinates
    elif key == ord('j'):
        borders.set_image(balloon, left_cam)
        print("Saved the %.0f point: (%.0f,%.0f)" % (borders.index, balloon.x, balloon.y))
        if borders.index == 4:
            borders.write_borders(BORDERS_FILENAME)
            drone_1.set_home((borders.x_middle_1, borders.y_middle))
            drone_2.set_home((borders.x_middle_2, borders.y_middle))

    # the 'r' button is set as the read text_color from file
    elif key == ord('r'):
        borders.read_borders(BORDERS_FILENAME)
        drone_1.set_home((borders.x_middle_1, borders.y_middle))
        drone_2.set_home((borders.x_middle_2, borders.y_middle))

    elif key == ord('z'):
        drone_1.testing = 1

    return True


def capture_video(drone_1: Drone, drone_2: Drone,  balloon: RecognizableObject, cameras_distance, left: Camera, right: Camera):

    continue_loop = True

    borders = Borders()
    drones = [drone_1, drone_2]
    recognizable_objects = [balloon] + drones
    #read_colors(COLORS_FILENAME, recognizable_objects)
    borders.read_borders(BORDERS_FILENAME)

    if borders.set_borders:
        drone_1.set_home((borders.x_middle_1, borders.y_middle))
        drone_2.set_home((borders.x_middle_2, borders.y_middle))

    drone_1.active = True
    
    while continue_loop:
        # Capture the video frame by frame
        if not left.capture():
            continue
        if not right.capture():
            continue

        for recognizable_object in recognizable_objects:
            # Process frames
            recognizable_object.detect_and_set_coordinates(left, right, cameras_distance)
            
        display_frames(recognizable_objects, left, right, borders)

        for i, drone in enumerate(drones):
            # State Machine
            state_kwargs = {'drone': drones[i], 'other drone': drones[i-1], 'balloon': balloon, 'borders': borders}
            drone.state.run(**state_kwargs)
            transition = drone.state.to_transition(**state_kwargs)
            if transition:
                drone.state.cleanup(transition, **state_kwargs)
                drone.state = drone.state.next(transition)
                drone.state.setup(**state_kwargs)

        continue_loop = interactive_loop(borders, left, balloon, drone_1, drone_2)

    for i in range(3):
        for drone in drones:
            drone.stop()

    if drone_1.tookoff:
        try:
            drone_1.land()
        except:
            pass

    if drone_2.tookoff:
        try:
            drone_2.land()
        except:
            pass

    # After the loop release the cap object
    left.release()
    right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


def main():
    right_cam = EFRAT_WEB
    left_cam = ORI_PHONE

    distance = 78.5
    capture_video(Drone(1, (0, 191, 255), iface_ip="192.168.10.2"), Drone(2, (38, 38, 200), iface_ip="192.168.10.10"), RecognizableObject((255, 54, 89), "balloon"), distance,
                  left_cam, right_cam)


if __name__ == "__main__":
    main()
