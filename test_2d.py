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


def interactive_loop(image_3d: Image3D, colors: ColorBounds, borders: Borders, loop_status: Status, left_cam: Camera, tello_1: Tello, tello_2: Tello) -> bool:
    key = cv2.waitKey(1) & 0xFF
    str_colors_changed = "color bounds changed"

    # the 'c' button reconnects to the drone
    if key == ord('c'):
        tello_1.connect()
        tello_2.connect()
        loop_status.reset()

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
        colors.drone_1_left.change(lower, upper)
        print(str_colors_changed)

    # the 'f' button is set as the detect color of drone in the right cam
    elif key == ord('f'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.drone_1_right.change(lower, upper)
        print(str_colors_changed)

    elif key == ord('t'):
        loop_status.takeoff()
        tello_1.connect()
        tello_2.connect()
        print("tello 1 battery = ", tello_1.get_battery(), "%")
        print("tello 2 battery = ", tello_2.get_battery(), "%")
        tello_1.takeoff()
        tello_2.takeoff()

    elif key == ord('y'):
        loop_status.start_track(image_3d.get_phys_drone(0), image_3d.get_phys_drone(1))

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        loop_status.stop_loop()
        return False

    # the 'l' button is set as the landing button
    elif key == ord('l'):
        loop_status.stop_loop()
        loop_status.state = ON_GROUND()

    # the 'h' button is set as the hitting balloon method
    elif key == ord("h"):
        loop_status.hit_mode_on()

    elif key == ord("w"):
        tello_1.flip_forward()
        tello_2.flip_forward()

    # the 'p' button is set as the save colors to file
    elif key == ord('p'):
        colors.write_colors(COLORS_FILENAME)

    # the 'k' button is set as the read colors from file
    elif key == ord('k'):
        colors.read_colors(COLORS_FILENAME)

    # the 'j' button is set as the saving the borders. can save 4 coordinates
    elif key == ord('j'):
        borders.set_image(image_3d, left_cam)
        print("saved the %.0f coordinate: (%.0f,%.0f,%.0f)" % (borders.index, image_3d.get_phys_balloon(0), image_3d.get_phys_balloon(1), image_3d.get_phys_balloon(2)))
        if borders.index == 4:
            borders.write_borders(BORDERS_FILENAME)

    # the 'b' button is set as the save borders to file
  #  elif key == ord('b'):
   #     borders.write_borders(BORDERS_FILENAME)

    # the 'r' button is set as the read colors from file
    elif key == ord('r'):
        borders.read_borders(BORDERS_FILENAME)
        print("middle is ({0:.3f},{1:.3f})".format(borders.x_middle_1, borders.y_middle_1))

    # the 'a' button is set to abort hitting state back to seek middle
    elif key == ord('a'):
        loop_status.stop_hit()


    return True


def capture_video(tello_1: Tello, tello_2: Tello, cameras_distance, left: Camera, right: Camera, colors: ColorBounds, borders: Borders, method='parallel'):
    vid_left = cv2.VideoCapture(left.index)
    vid_right = cv2.VideoCapture(right.index)

    frame_counter = 0
    image_old = None
    continue_test = True

    loop_status = Status()
    old_images_vel = [None]*10
    old_images_coord = [None]*4

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
        if frame_counter > len(old_images_vel):
            image_now.detect_all(colors, image_old)
            balloon_exist, drone_1_exist, drone_2_exist = image_now.calculate_all_distances(left, right, cameras_distance, method=method)
            if not balloon_exist:
                image_now.phys_x_balloon, image_now.phys_y_balloon = image_old.get_phys_balloon(0), image_old.get_phys_balloon(1), image_old.get_phys_balloon(2)
            if not drone_1_exist:
                image_now.phys_x_drone_1, image_now.phys_y_drone_1 = image_old.get_phys_drone_1(0), image_old.get_phys_drone_1(1), image_old.get_phys_drone_1(2)
            if not drone_2_exist:
                image_now.phys_x_drone_2, image_now.phys_y_drone_2 = image_old.get_phys_drone_2(0), image_old.get_phys_drone_2(1), image_old.get_phys_drone_2(2)

            image_now.calculate_mean_velocities(old_images_vel)
        
        text_balloon_coor = "c(%.0f,%.0f,%.0f)" % (image_now.phys_x_balloon, image_now.phys_y_balloon, image_now.phys_z_balloon)
        text_drone_1_coor = "c(%.0f,%.0f,%.0f)" % (image_now.phys_x_drone_1, image_now.phys_y_drone_1, image_now.phys_z_drone_1)
        text_drone_2_coor = "c(%.0f,%.0f,%.0f)" % (image_now.phys_x_drone_2, image_now.phys_y_drone_2, image_now.phys_z_drone_2)
        text_balloon_vel = "v(%.0f,%.0f)" % (image_now.velocity_x_balloon, image_now.velocity_y_balloon)
        text_drone_1_vel = "v(%.0f,%.0f)" % (image_now.velocity_x_drone_1, image_now.velocity_y_drone_1)
        text_drone_2_vel = "v(%.0f,%.0f)" % (image_now.velocity_x_drone_2, image_now.velocity_y_drone_2)

        # Display the resulting frame
        left_img = image_now.frame_left.image_to_show("left", text_balloon=text_balloon_coor, text_drone_1=text_drone_1_coor, text_drone_2=text_drone_2_coor, text_color=(150,250,200))
        left_img = borders.draw_borders(left_img, image_now, color_in=(0, 240, 0), color_out=(0, 0, 240))
        left_img = image_with_circle(left, left_img, loop_status.dest_coords, rad_phys=5, thickness=2)
        cv2.imshow("left", left_img)
        image_now.frame_right.show_image("right", text_balloon=text_balloon_vel, text_drone_1=text_drone_1_vel, text_drone_2=text_drone_2_vel, text_color=(240,150,240))

        draw_xy_display(borders, image_now.phys_x_balloon, image_now.phys_y_balloon, image_now.phys_x_drone_1, image_now.phys_y_drone_2, loop_status.dest_coords[0], loop_status.dest_coords[1])

        state.run(**{'image_3d': image_now, 'loop_status': loop_status, 'tello': tello, 'borders': borders})
        transition = state.to_transition(**{'image_3d': image_now, 'loop_status': loop_status, 'tello': tello})
        if transition:
            loop_status.state = state.next(transition)


        old_images_vel[frame_counter % len(old_images_vel)] = image_now
        old_images_coord[frame_counter % len(old_images_coord)] = image_now
        image_old = image_now
    
        continue_test = interactive_loop(image_now, colors, borders, loop_status, left, tello_1, tello_2)
        if not loop_status.continue_loop:
            break
    
    print("Hit Time: " + str(loop_status.end_hit_timer - loop_status.start_hit_timer))
    if loop_status.tookoff:
        tello_1.land()
        tello_2.land()
        print("tello 1 battery = ", tello_1.get_battery(), "%")
        print("tello 2 battery = ", tello_2.get_battery(), "%")

    # After the loop release the cap object
    vid_left.release()
    vid_right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

    return continue_test, colors


if __name__ == "__main__":
    tello_1 = Tello(iface_ip="192.168.10.2")
    tello_2 = Tello(iface_ip="192.168.10.3")

    colors = ColorBounds()
    borders = Borders()
    continue_test = True

    left = NIR_PHONE_NIR
    right = MAYA_PHONE_NIR

    distance = 58
    while continue_test:
        continue_test, colors = capture_video(tello_1, tello_2, distance, left, right, colors, borders, method='parallel')
