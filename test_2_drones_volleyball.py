import cv2, numpy as np
from consts import *
from recognizable_object import RecognizableObject
from drone import Drone
from common import load_colors, save_colors, display_frames
from borders import Borders
from camera import Camera
import faulthandler

def interactive_loop(borders: Borders, left_cam: Camera, balloon: RecognizableObject, drone_1: Drone, drone_2: Drone) -> bool:
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

    # the 'w' button is set as the detect color of drone_2 in the left_cam cam
    elif key == ord('w'):
        drone_2.detect_color(True)
        print(str_colors_changed)

    # the 'e' button is set as the detect color of drone_2 in the right_cam cam
    elif key == ord('e'):
        drone_2.detect_color(False)
        print(str_colors_changed)

    elif key == ord('t'):
        drone_1.tookoff = True
        drone_2.tookoff = True

    elif key == ord('y'):
        drone_1.start_track()
        drone_2.start_track()

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        return False

    # the 'p' button is set as the save text_color to file
    elif key == ord('p'):
        save_colors(COLORS_FILENAME, [balloon, drone_1.recognizable_object, drone_2.recognizable_object])

    # the 'k' button is set as the read text_color from file
    elif key == ord('k'):
        load_colors(COLORS_FILENAME, [balloon, drone_1.recognizable_object, drone_2.recognizable_object])

    # the 'j' button is set as the saving the borders. can save 4 coordinates
    elif key == ord('j'):
        borders.set_image(balloon, left_cam)
        print("Saved the %.0f point: (%.0f,%.0f)" % (borders.index, balloon.x, balloon.y))
        if borders.index == 4:
            borders.save_borders(BORDERS_FILENAME)
            drone_1.set_home((borders.x_middle_1, borders.y_middle))
            drone_2.set_home((borders.x_middle_2, borders.y_middle))
            # drone_2.default_height = drone_q.default_height + 30 # for safety we can delete this when seek middle works

    # the 'r' button is set as the read text_color from file
    elif key == ord('r'):
        borders.load_borders(BORDERS_FILENAME)
        drone_1.set_home((borders.x_middle_1, borders.y_middle))
        drone_2.set_home((borders.x_middle_2, borders.y_middle))

    elif key == ord('z'):
        drone_1.testing = 1
        drone_2.testing = 1

    return True


def capture_video(drone_1: Drone, drone_2: Drone,  balloon: RecognizableObject, cameras_distance, left: Camera, right: Camera):

    continue_loop = True

    borders = Borders()
    drones = [drone_1, drone_2]
    recognizable_objects = [balloon] + [drone.recognizable_object for drone in drones]
    load_colors(COLORS_FILENAME, recognizable_objects)
    borders.load_borders(BORDERS_FILENAME, left)

    if borders.set_borders:
        drone_1.set_home((borders.x_middle_1, borders.y_middle))
        drone_2.set_home((borders.x_middle_2, borders.y_middle))
    
    drone_1.active = True
    drone_1.set_home((90, 350))
    drone_2.set_home((7, 365))
    
    while continue_loop:
        # Capture the video frame by frame
        if not left.capture():
            continue
        if not right.capture():
            continue

        for recognizable_object in recognizable_objects:
            # Process frames
            recognizable_object.detect_and_set_coordinates(left, right, cameras_distance)
            
        display_frames(balloon, drones, left, right, borders)

        # Set Obstacle
        for drone in drones:
            if drone.start:
                drone.set_obstacle(left)

        # State Machine
        for i, drone in enumerate(drones):
            drone.state.run(drone, drones[1-i], balloon, borders)
            transition = drone.state.to_transition(drone, drones[1-i], balloon, borders)
            if transition:
                drone.state.cleanup(transition, drone, drones[1-i], balloon, borders)
                drone.state = drone.state.next(transition)
                print("drone", drone.ident, " state:", drone.state)
                drone.state.setup(drone, drones[1-i], balloon, borders)

        continue_loop = interactive_loop(borders, left, balloon, drone_1, drone_2)

    for i in range(3):
        for drone in drones:
            drone.stop()

    for drone in drones:
        if drone.tookoff:
            try:
                drone.land()
            except:
                pass

    # After the loop release the cap object
    left.release()
    right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


def main():
    right_cam = C920_NIR_2
    left_cam = C920_NIR_1

    drone_1 = Drone(1, (0, 191, 255), 7, iface_ip="192.168.10.2")
    drone_2 = Drone(2, (38, 38, 200), 7, iface_ip="192.168.10.10")
    balloon = RecognizableObject((255, 54, 89), 11.3, "balloon")

    distance = 111.9
    capture_video(drone_1, drone_2, balloon, distance, left_cam, right_cam)


if __name__ == "__main__":
    faulthandler.enable()
    main()
