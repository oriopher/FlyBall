import cv2, numpy as np
from consts import *
from recognizable_object import RecognizableObject
from drone import Drone
from common import load_colors, save_colors, display_frames
from borders import Borders
from camera import Camera
import faulthandler
from gui import Gui

def interactive_loop(borders: Borders, gui: Gui, left_cam: Camera, right_cam: Camera, cam_distance, balloon: RecognizableObject, drone_1: Drone, drone_2: Drone) -> bool:
    event, values = gui.window.read(timeout=1)
    str_colors_changed = "Color bounds changed"
    distance = cam_distance

    # the 'v' button is set as the detect color of recognizable_object in the left_cam cam
    if event == 'v' or event == 'Balloon color (L)':
        balloon.detect_color(True)
        print(str_colors_changed)

    # the 'n' button is set as the detect color of recognizable_object in the right_cam cam
    elif event == 'n' or event == 'Balloon color (R)':
        balloon.detect_color(False)
        print(str_colors_changed)

    # the 's' button is set as the detect color of drone_1 in the left_cam cam
    elif event == 's' or event == 'Drone 1 color (L)':
        drone_1.detect_color(True)
        print(str_colors_changed)

    # the 'f' button is set as the detect color of drone_1 in the right_cam cam
    elif event == 'f' or event == 'Drone 1 color (R)':
        drone_1.detect_color(False)
        print(str_colors_changed)

    # the 'w' button is set as the detect color of drone_2 in the left_cam cam
    elif event == 'w' or event == 'Drone 2 color (L)':
        drone_2.detect_color(True)
        print(str_colors_changed)

    # the 'e' button is set as the detect color of drone_2 in the right_cam cam
    elif event == 'e' or event == 'Drone 2 color (R)':
        drone_2.detect_color(False)
        print(str_colors_changed)

    elif event == 't' or event == 'Take Off':
        drone_1.tookoff = True
        drone_2.tookoff = True

    elif event == 'y' or event == 'Go Home':
        drone_1.start_track()
        drone_2.start_track()

    # the 'q' button is set as the quitting button
    elif event == 'q' or event == 'Quit' or event == Gui.WINDOW_CLOSED:
        return False, distance

    # the 'p' button is set as the save text_colors to file
    elif event == 'p' or event == 'Save Colors':
        save_colors(COLORS_FILENAME, [balloon, drone_1.recognizable_object, drone_2.recognizable_object])

    # the 'k' button is set as the read text_colors from file
    elif event == 'k' or event == 'Load Colors':
        load_colors(COLORS_FILENAME, [balloon, drone_1.recognizable_object, drone_2.recognizable_object])

    # the 'j' button is set as the saving the borders. can save 4 coordinates
    elif event == 'j' or event == 'Set Borders':
        borders.set_image(balloon, left_cam)
        print("Saved the %.0f coordinate: (%.0f,%.0f)" % (borders.index, balloon.x, balloon.y))
        gui.update_borders_button(borders.index)
        if borders.index == 4:
            borders.save_borders(BORDERS_FILENAME)
            drone_1.set_home((borders.x_middle_1, borders.y_middle))
            drone_2.set_home((borders.x_middle_2, borders.y_middle))
            gui.show_homes_gui(drone_1.home, drone_2.home)
            # drone_2.default_height = drone_q.default_height + 30 # for safety we can delete this when seek middle works

    # the 'r' button is set as the read text_colors from file
    elif event == 'r' or event == 'Load Borders':
        borders.load_borders(BORDERS_FILENAME)
        drone_1.set_home((borders.x_middle_1, borders.y_middle))
        drone_2.set_home((borders.x_middle_2, borders.y_middle))
        gui.show_homes_gui(drone_1.home, drone_2.home)

    elif event == 'z' or event == 'Start Play':
        drone_1.testing = 1
        drone_2.testing = 1

    elif event == 'flip_left':
        left_cam.flip = -1 if values['flip_left'] else 1
        left_cam.is_flipped = values['flip_left']

    elif event == 'flip_right':
        right_cam.flip = -1 if values['flip_right'] else 1
        right_cam.is_flipped = values['flip_right']

    elif event == 'recieved_input':
        distance = gui.update_val(left_cam, right_cam, drone_1, drone_2, distance, values)

    elif event == 'SLIDER_H' or event == 'SLIDER_S' or event == 'SLIDER_V' or event == 'SLIDER_T':
        recognizable_objects = [balloon] + [drone_1.recognizable_object, drone_2.recognizable_object]
        gui.update_hsv(recognizable_objects, event, values)
    
    return True, distance


def capture_video(drone_1: Drone, drone_2: Drone,  balloon: RecognizableObject, cameras_distance, left: Camera, right: Camera):

    continue_loop = True

    borders = Borders()
    gui = Gui()
    gui.show_gui()

    drones = [drone_1, drone_2]
    recognizable_objects = [balloon] + [drone.recognizable_object for drone in drones]
    load_colors(COLORS_FILENAME, recognizable_objects)
    borders.load_borders(BORDERS_FILENAME, left)

    if borders.set_borders:
        drone_1.set_home((borders.x_middle_1, borders.y_middle))
        drone_2.set_home((borders.x_middle_2, borders.y_middle))
        gui.show_homes_gui(drone_1.home, drone_2.home)

    drone_1.active = True
    drone_1.set_home((90, 350))
    drone_2.set_home((7, 365))
    gui.show_homes_gui(drone_1.home, drone_2.home)    

    while continue_loop:
        # Capture the video frame by frame
        if not left.capture():
            continue
        if not right.capture():
            continue

        for recognizable_object in recognizable_objects:
            # Process frames
            recognizable_object.detect_and_set_coordinates(left, right, cameras_distance)
            
        gui.display_frames_gui(balloon, drones, left, right, borders)

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
        gui.update_states(str(drone_1.state), str(drone_2.state))

        continue_loop, distance = interactive_loop(borders, gui, left, right, cameras_distance, balloon, drone_1, drone_2)

    for i in range(3):
        for drone in drones:
            if drone.tookoff:
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

    right_cam = Camera(56, 39, 2, False)
    left_cam = Camera(56, 39, 0, False)

    drone_1 = Drone(1, (0, 191, 255), iface_ip="192.168.10.10")
    drone_2 = Drone(2, (38, 38, 200), iface_ip="192.168.10.2")
    balloon = RecognizableObject((255, 54, 89), "balloon")

    distance = 111.9
    capture_video(drone_1, drone_2, balloon, distance, left_cam, right_cam)


if __name__ == "__main__":
    faulthandler.enable()
    main()
