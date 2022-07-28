from datetime import datetime, timedelta
import numpy as np
import cv2
from borders import Borders
from color_bounds import ColorBounds
from image_3d import Image3D
from loop_status import Status
from djitellopy import Tello
from camera import Camera
from loop_state_machine import ON_GROUND
from velocity_pot import lin_velocity_with_two_params, track_balloon
from utils import image_with_circle, FLOOR_HEIGHT, DRONE_DEFAULT_HEIGHT

ORI_WEB = Camera(51.3, 0, False)
ORI_PHONE = Camera(66.9, 3, False)
NIR_PHONE = Camera(65, 0, False)
MAYA_WEB = Camera(61, 0, True)
EFRAT_WEB = Camera(61, 2, False)
EFRAT_PHONE = Camera(64, 3, False)
MAYA_PHONE = Camera(63, 2, False)

NIR_PHONE_NIR = Camera(67, 0, False)
EFRAT_PHONE_NIR = Camera(77, 2, False)

COLORS_FILENAME = "color_bounds.txt"
BORDERS_FILENAME = "borders.txt"

def hit_ball_rc(image_3d: Image3D, tello: Tello, loop_status: Status):
    UPPER_LIMIT = 200
    LOWER_LIMIT = 0
    XY_LIMIT = 10
    Z_LIMIT = 15
    VEL_LIMIT = 5

    x_rel = int(image_3d.get_phys_balloon(0) - image_3d.get_phys_drone(0))
    y_rel = int(image_3d.get_phys_balloon(1) - image_3d.get_phys_drone(1))
    z_rel = int(loop_status.hit_coords[2] - image_3d.get_phys_drone(2))

    if abs(x_rel) < XY_LIMIT \
            and abs(y_rel) < XY_LIMIT \
            and LOWER_LIMIT < z_rel < UPPER_LIMIT \
            and abs(image_3d.velocity_x_drone) < VEL_LIMIT \
            and abs(image_3d.velocity_y_drone) < VEL_LIMIT:
        if z_rel < Z_LIMIT:
            left_right, for_back = 0, 0
            up_down = -100
            while not tello.send_rc_control:
                continue
            tello.send_rc_control(left_right, for_back, up_down, 0)
            loop_status.hit_mode_off()
            return

        left_right = lin_velocity_with_two_params(x_rel, image_3d.velocity_x_balloon, 'x')
        for_back = lin_velocity_with_two_params(y_rel, image_3d.velocity_y_balloon, 'y')
        up_down = 100
        while not tello.send_rc_control:
            continue
        tello.send_rc_control(left_right, for_back, up_down, 0)

    else:
        track_balloon(image_3d, tello)



def interactive_loop(image_3d: Image3D, colors: ColorBounds, borders: Borders, loop_status: Status, left_cam: Camera, tello: Tello) -> bool:
    key = cv2.waitKey(1) & 0xFF
    str_colors_changed = "color bounds changed"

    # the 'c' button reconnects to the drone
    if key == ord('c'):
        tello.connect()
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
        loop_status.stop_loop()
        return False

    # the 'l' button is set as the landing button
    elif key == ord('l'):
        loop_status.stop_loop()
        loop_status.state = ON_GROUND()

    # the 'h' button is set as the hitting balloon method
    elif key == ord("h"):
        coords = (image_3d.phys_x_balloon, image_3d.phys_y_balloon, image_3d.phys_z_balloon)
        loop_status.hit_mode_on(coords)

    elif key == ord("w"):
        tello.flip_forward()

    # the 'p' button is set as the save colors to file
    elif key == ord('p'):
        colors.write_colors(COLORS_FILENAME)

    # the 'k' button is set as the read colors from file
    elif key == ord('k'):
        colors.read_colors(COLORS_FILENAME)

    # the 'j' button is set as the saving the borders. can save 4 coordinates
    elif key == ord('j'):
        borders.set_image(image_3d, left_cam)
        print("saved the %.0f coordinate: (%.0f,%.0f,%.0f)" % (borders.index, image_3d.phys_x_balloon, image_3d.phys_y_balloon, image_3d.phys_z_balloon))
        if borders.index == 4:
            borders.write_borders(BORDERS_FILENAME)

    # the 'b' button is set as the save borders to file
  #  elif key == ord('b'):
   #     borders.write_borders(BORDERS_FILENAME)

    # the 'r' button is set as the read colors from file
    elif key == ord('r'):
        borders.read_borders(BORDERS_FILENAME)
        print("middle is ({0:.3f},{1:.3f})".format(borders.x_middle, borders.y_middle))

    # the 'a' button is set to abort hitting state back to seek middle
    elif key == ord('a'):
        loop_status.stop_hit()


    return True


def capture_video(tello: Tello, cameras_distance, left: Camera, right: Camera, colors: ColorBounds, borders: Borders, method='parallel'):
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
            balloon_exist, drone_exist = image_now.calculate_all_distances(left, right, cameras_distance, method=method)
            if not balloon_exist:
                image_now.phys_x_balloon, image_now.phys_y_balloon, image_now.phys_z_balloon = image_old.get_phys_balloon(0), image_old.get_phys_balloon(1), image_old.get_phys_balloon(2)
            if not drone_exist:
                image_now.phys_x_drone, image_now.phys_y_drone, image_now.phys_z_drone = image_old.get_phys_drone(0), image_old.get_phys_drone(1), image_old.get_phys_drone(2)

            image_now.calculate_mean_velocities(old_images_vel)
        
        text_balloon_coor = "c(%.0f,%.0f,%.0f)" % (image_now.phys_x_balloon, image_now.phys_y_balloon, image_now.phys_z_balloon)
        text_drone_coor = "c(%.0f,%.0f,%.0f)" % (image_now.phys_x_drone, image_now.phys_y_drone, image_now.phys_z_drone)
        text_balloon_vel = "v(%.0f,%.0f)" % (image_now.velocity_x_balloon, image_now.velocity_y_balloon)
        text_drone_vel = "v(%.0f,%.0f)" % (image_now.velocity_x_drone, image_now.velocity_y_drone)
    
        # Display the resulting frame
        left_img = image_now.frame_left.image_to_show("left", text_balloon=text_balloon_coor, text_drone=text_drone_coor, text_color=(150,250,200))
        left_img = borders.draw_borders(left_img, image_now, color_in=(0, 240, 0), color_out=(0, 0, 240))
        left_img = image_with_circle(left, left_img, loop_status.dest_coords, rad_phys=5, thickness=2)
        cv2.imshow("left", left_img)
        image_now.frame_right.show_image("right", text_balloon=text_balloon_vel, text_drone=text_drone_vel, text_color=(240,150,240))

        state.run(**{'image_3d': image_now, 'loop_status': loop_status, 'tello': tello, 'borders': borders})
        transition = state.to_transition(**{'image_3d': image_now, 'loop_status': loop_status, 'tello': tello})
        if transition:
            loop_status.state = state.next(transition)

        # balloon is out of borders. drone is seeking the middle until the balloon is back
        # if loop_status.first_seek and (not borders.balloon_in_borders(image_now) or not loop_status.start):
        #     print("seek middle")
        #     loop_status.stop_hit()

        # balloon returned to the play area, we can continue to play
        #if borders.in_borders(image_now) and not loop_status.start_track:
        #   loop_status.out_of_borders = False
        #   loop_status.start_track = True

        old_images_vel[frame_counter % len(old_images_vel)] = image_now
        old_images_coord[frame_counter % len(old_images_coord)] = image_now
        image_old = image_now
    
        continue_test = interactive_loop(image_now, colors, borders, loop_status, left, tello)
        if not loop_status.continue_loop:
            break
    
    print("Hit Time: " + str(loop_status.end_hit_timer - loop_status.start_hit_timer))
    if loop_status.tookoff:
        tello.land()
        print("battery = ", tello.get_battery(), "%")

    # After the loop release the cap object
    vid_left.release()
    vid_right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

    return continue_test, colors


if __name__ == "__main__":
    tello = Tello()

    colors = ColorBounds()
    borders = Borders()
    continue_test = True

    left = NIR_PHONE_NIR
    right = MAYA_PHONE

    distance = 58
    while continue_test:
        continue_test, colors = capture_video(tello, distance, left, right, colors, borders, method='parallel')
