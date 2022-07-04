from datetime import datetime
import numpy as np
import cv2
from color_bounds import ColorBounds
from image_3d import Image3D
from loop_status import Status
from djitellopy import Tello
from camera import Camera
from velocity_pot import lin_velocity_with_two_params, track_balloon
from loop_state_machine import State, ON_GROUND



ORI_WEB = Camera(51.3, 0, False)
ORI_PHONE = Camera(66.9, 3, False)
NIR_PHONE = Camera(67, 4, False)
MAYA_WEB = Camera(61, 0, True)
EFRAT_WEB = Camera(61, 0, True)

COLORS_FILENAME = "color_bounds.txt"

FLOOR_HEIGHT = -50
DRONE_DEFAULT_HEIGHT = FLOOR_HEIGHT + 50


def hit_ball_rc(image_3d: Image3D, tello: Tello, loop_status: Status):
    UPPER_LIMIT = 200
    LOWER_LIMIT = 0
    XY_LIMIT = 10
    Z_LIMIT = 15
    VEL_LIMIT = 5

    x_rel = int(image_3d.phys_x_balloon - image_3d.phys_x_drone)
    y_rel = int(image_3d.phys_y_balloon - image_3d.phys_y_drone)
    z_rel = int(loop_status.hit_coords[2] - image_3d.phys_z_drone)

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
            loop_status.set_hit_time()
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



def interactive_loop(image_3d: Image3D, colors: ColorBounds, loop_status: Status, tello: Tello) -> bool:
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
        loop_status.start_track()

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

    return True


def capture_video(tello: Tello, cameras_distance, left: Camera, right: Camera, colors: ColorBounds, method='parallel'):
    vid_left = cv2.VideoCapture(left.index)
    vid_right = cv2.VideoCapture(right.index)

    frame_counter = 0
    image_old = None
    continue_test = True

    loop_status = Status()
    old_images = [None]*15

    while(True):
        state = loop_status.state
        frame_counter = frame_counter+1
        # Capture the video frame by frame
        ret_left, image_left = vid_left.read()
        ret_right, image_right = vid_right.read()
        image_now = Image3D(image_left, image_right)
    
        # Process frames
        if frame_counter > len(old_images):
            image_now.detect_all(colors, image_old)
            balloon_exist, drone_exist = image_now.calculate_all_distances(left, right, cameras_distance, method=method)
            if not balloon_exist:
                image_now.phys_x_balloon, image_now.phys_y_balloon = image_old.phys_x_balloon, image_old.phys_y_balloon
            if not drone_exist:
                image_now.phys_x_drone, image_now.phys_y_drone = image_old.phys_x_drone, image_old.phys_y_drone

            image_now.calculate_mean_velocities(old_images)
        
        text_balloon_coor = "c(%.0f,%.0f,%.0f)" % (image_now.phys_x_balloon, image_now.phys_y_balloon, image_now.phys_z_balloon)
        text_drone_coor = "c(%.0f,%.0f,%.0f)" % (image_now.phys_x_drone, image_now.phys_y_drone, image_now.phys_z_drone)
        text_balloon_vel = "v(%.0f,%.0f)" % (image_now.velocity_x_balloon, image_now.velocity_y_balloon)
        text_drone_vel = "v(%.0f,%.0f)" % (image_now.velocity_x_drone, image_now.velocity_y_drone)
    
        # Display the resulting frame
        image_now.frame_left.show_image("left", text_balloon=text_balloon_coor, text_drone=text_drone_coor, text_color=(150,250,200))
        image_now.frame_right.show_image("right", text_balloon=text_balloon_vel, text_drone=text_drone_vel, text_color=(240,150,240))

        state.run(**{'image_3d': image_now, 'loop_status': loop_status, 'tello': tello})
        if state.to_transition(**{'image_3d': image_now, 'loop_status': loop_status, 'tello': tello}):
            loop_status.state = state.next


        # if loop_status.hit_mode():
        #     hit_ball_rc(image_now, tello, loop_status)
        # elif datetime.datetime.now() - datetime.timedelta(seconds = 1) > loop_status.hit_time:
        #     continue
        # elif loop_status.start_track:
        #     track_balloon(image_now, tello)

        old_images[frame_counter % len(old_images)] = image_now
        image_old = image_now
   
        continue_test = interactive_loop(image_now, colors, loop_status, tello)
        if not loop_status.continue_loop:
            break
    
    if loop_status.tookoff:
        tello.land()
        print("battery = ", tello.get_battery(), "%")

    # After the loop release the cap object
    vid_left.release()
    vid_right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

    return continue_test, colors


# return how much cm in one pixel.
def pixels_to_cm(distance, num_pixels, fov_angle):  
    return distance * 2 * np.tan(fov_angle/2) * fov_angle / num_pixels


if __name__ == "__main__":
    tello = Tello()

    colors = ColorBounds()
    continue_test = True

    left = ORI_PHONE
    right = NIR_PHONE

    distance = 60
    while continue_test:
        continue_test, colors = capture_video(tello, distance, left, right, colors, method='parallel')
