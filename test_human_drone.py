import numpy as np
import cv2
from borders import Borders
from xy_display import draw_xy_display
from color_bounds import ColorBounds
from image_3d import Image3D
from loop_status import Status
from djitellopy import Tello
from camera import Camera
from loop_state_machine import ON_GROUND
from common import *


def interactive_loop(image_3d: Image3D, colors: ColorBounds, borders: Borders, loop_status: Status, left_cam: Camera, tello: Tello) -> bool:
    key = cv2.waitKey(1) & 0xFF
    str_colors_changed = "Color bounds changed"

    # the 'v' button is set as the detect color of balloon in the left cam
    if key == ord('v'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.ball_left.change(lower, upper)
        print(str_colors_changed)

    # the 'n' button is set as the detect color of balloon in the right cam
    elif key == ord('n'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.ball_right.change(lower, upper)
        print(str_colors_changed)

    # the 's' button is set as the detect color of drone in the left cam
    elif key == ord('s'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.drone_left.change(lower, upper)
        print(str_colors_changed)

    # the 'f' button is set as the detect color of drone in the right cam
    elif key == ord('f'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.drone_right.change(lower, upper)
        print(str_colors_changed)

    elif key == ord('t'):
        loop_status.takeoff()
        tello.connect()
        print("battery = ", tello.get_battery(), "%")
        tello.takeoff()

    elif key == ord('y'):
        loop_status.start_track(image_3d.get_phys_drone(0), image_3d.get_phys_drone(1))

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        return False

    # the 'p' button is set as the save colors to file
    elif key == ord('p'):
        colors.write_colors(COLORS_FILENAME)

    # the 'k' button is set as the read colors from file
    elif key == ord('k'):
        colors.read_colors(COLORS_FILENAME)

    # the 'j' button is set as the saving the borders. can save 4 coordinates
    elif key == ord('j'):
        borders.set_image(image_3d, left_cam)
        print("Saved the %.0f coordinate: (%.0f,%.0f)" % (borders.index, image_3d.get_phys_balloon(0), image_3d.get_phys_balloon(1)))
        if borders.index == 4:
            borders.write_borders(BORDERS_FILENAME)

    # the 'r' button is set as the read colors from file
    elif key == ord('r'):
        borders.read_borders(BORDERS_FILENAME)

    # the 'a' button is set to abort hitting state back to seek middle
    elif key == ord('a'):
        loop_status.stop_hit()

    elif key == ord('z'):
        loop_status.testing = 1

    return True


def display_frames(image_now : Image3D, loop_status, borders : Borders):
    text_balloon_coor = "c(%.0f,%.0f,%.0f)" % (image_now.get_phys_balloon(0), image_now.get_phys_balloon(1), image_now.get_phys_balloon(2))
    text_drone_coor = "c(%.0f,%.0f,%.0f)" % (image_now.get_phys_drone(0), image_now.get_phys_drone(1), image_now.get_phys_drone(2))
    text_balloon_vel = "v(%.0f,%.0f,%.0f)" % (image_now.velocity_x_balloon, image_now.velocity_y_balloon, image_now.velocity_z_balloon)
    text_drone_vel = "v(%.0f,%.0f,%.0f)" % (image_now.velocity_x_drone, image_now.velocity_y_drone, image_now.velocity_z_drone)

    left_img = image_now.frame_left.image_to_show("left", text_balloon=text_balloon_coor, text_drone=text_drone_coor, text_color=(150,250,200))
    left_img = borders.draw_borders(left_img, image_now, color_in=(0, 240, 0), color_out=(0, 0, 240))
    left_img = image_with_circle(left, left_img, loop_status.dest_coords, rad_phys=7, thickness=2)
    cv2.imshow("left", left_img)
    image_now.frame_right.show_image("right", text_balloon=text_balloon_vel, text_drone=text_drone_vel, text_color=(240,150,240))
    draw_xy_display(borders, image_now.phys_x_balloon, image_now.phys_y_balloon, image_now.phys_x_drone, image_now.phys_y_drone, loop_status.dest_coords[0], loop_status.dest_coords[1])



def capture_video(tello: Tello, cameras_distance, left: Camera, right: Camera, method='parallel'):
    vid_left = cv2.VideoCapture(left.index)
    vid_right = cv2.VideoCapture(right.index)

    frame_counter = 0
    image_old = None
    continue_test = True

    loop_status = Status()
    colors = ColorBounds()
    borders = Borders()
    colors.read_colors(COLORS_FILENAME)
    borders.read_borders(BORDERS_FILENAME)
    old_images = [None]*10

    while(True):
        state = loop_status.state
        frame_counter = frame_counter+1
        # Capture the video frame by frame
        ret_left, image_left = vid_left.read()
        if not ret_left:
            continue
        real_image_left = image_left
        ret_right, image_right = vid_right.read()
        if not ret_right:
            continue
        image_now = Image3D(real_image_left, image_right)
    
        # Process frames
        if frame_counter > len(old_images):
            image_now.process_image(image_old, colors, left, right, cameras_distance, old_images, method)
        
        display_frames(image_now, loop_status, borders)

        state.run(**{'image_3d': image_now, 'loop_status': loop_status, 'tello': tello, 'borders': borders})
        transition = state.to_transition(**{'image_3d': image_now, 'loop_status': loop_status, 'tello': tello, 'borders': borders})
        if transition:
            loop_status.state = state.next(transition)

        old_images[frame_counter % len(old_images)] = image_now
        image_old = image_now
    
        continue_test = interactive_loop(image_now, colors, borders, loop_status, left, tello)
        if not continue_test:
            break
    
    if loop_status.tookoff:
        tello.land()
        print("battery = ", tello.get_battery(), "%")

    # After the loop release the cap object
    vid_left.release()
    vid_right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


if __name__ == "__main__":
    tello = Tello()

    continue_test = True

    left = MAYA_PHONE_NIR
    right = EFRAT_PHONE_NIR

    distance = 69
    capture_video(tello, distance, left, right, method='parallel')
