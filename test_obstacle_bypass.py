import cv2
from recognizable_object import RecognizableObject
from drone import Drone
from common import load_colors, save_colors, display_frames, BORDERS_FILENAME, COLORS_FILENAME, EFRAT_WEB, NIR_PHONE_NIR
from borders import Borders
from camera import Camera
from obstacle import Obstacle
import numpy as np



def interactive_loop(borders: Borders, obstacle: Obstacle, left_cam: Camera, balloon: RecognizableObject, drone: Drone) -> bool:
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

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        return False

    # the 'p' button is set as the save text_colors to file
    elif key == ord('p'):
        save_colors(COLORS_FILENAME, [balloon, drone])

    # the 'k' button is set as the read text_colors from file
    elif key == ord('k'):
        load_colors(COLORS_FILENAME, [balloon, drone])

    # the 'j' button is set as the saving the borders. can save 4 coordinates
    elif key == ord('j'):
        borders.set_image(balloon, left_cam)
        print("Saved the %.0f coordinate: (%.0f,%.0f)" % (borders.index, balloon.x, balloon.y))
        if borders.index == 4:
            borders.save_borders(BORDERS_FILENAME)
            drone.set_home((borders.x_middle, borders.y_middle))

    # the 'r' button is set as the read text_colors from file
    elif key == ord('r'):
        borders.load_borders(BORDERS_FILENAME)
        drone.set_home((borders.x_middle, borders.y_middle))

    elif key == ord('b'):
        obstacle.update_obstacle(balloon.x, balloon.y, balloon.x + np.random.randint(2, 15), balloon.y + np.random.randint(2, 15), balloon, drone, left_cam)

    elif key == ord('d'):     
        obstacle.update_obstacle(drone.x, drone.y, drone.x + np.random.randint(2, 15), drone.y + np.random.randint(2, 15), drone, balloon, left_cam)

    return True


def capture_video(drone: Drone, balloon: RecognizableObject, cameras_distance, left: Camera, right: Camera):

    continue_loop = True

    borders = Borders()
    obstacle = Obstacle()
    recognizable_objects = [balloon, drone]
    load_colors(COLORS_FILENAME, recognizable_objects)
    borders.load_borders(BORDERS_FILENAME)
    if borders.set_borders:
        drone.set_home((borders.x_middle, borders.y_middle))

    while continue_loop:
        # Capture the video frame by frame
        if not left.capture():
            continue
        if not right.capture():
            continue

        for recognizable_object in recognizable_objects:
            # Process frames
            recognizable_object.detect_and_set_coordinates(left, right, cameras_distance)
            
        # updates start point of obstacle
        if obstacle.active == balloon:
            obstacle.update_start(balloon.x, balloon.y)
        if obstacle.active == drone:
            obstacle.update_start(drone.x, drone.y)            

        display_frames(balloon, drone, left, right, borders, obstacle)

        continue_loop = interactive_loop(borders, obstacle, left, balloon, drone)
    

    # After the loop release the cap object
    left.release()
    right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


def main():
    left_cam = NIR_PHONE_NIR
    right_cam = EFRAT_WEB

    distance = 74
    capture_video(Drone(1, (0, 191, 255), iface_ip="192.168.10.2"), RecognizableObject((255, 54, 89), "balloon"), distance,
                  left_cam, right_cam)


if __name__ == "__main__":
    main()
