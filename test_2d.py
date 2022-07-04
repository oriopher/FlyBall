import numpy as np
import cv2
from color_bounds import ColorBounds
from image_3d import Image3D
from loop_status import Status
from djitellopy import Tello
from camera import Camera
from time import sleep

ORI_WEB = Camera(51.3, 0, False)
ORI_PHONE = Camera(66.9, 3, False)
NIR_PHONE = Camera(67, 4, False)
MAYA_WEB = Camera(61, 0, True)
EFRAT_WEB = Camera(61, 0, True)

COLORS_FILENAME = "color_bounds.txt"

FLOOR_HEIGHT = -50
DRONE_DEFAULT_HEIGHT = FLOOR_HEIGHT + 50

def lin_velocity_with_acc(cm_rel, real_vel):
    #this function assumes the drone is looking at the cameras.
    a = 1.5
    b = 1
    stoping_limit = 5
    max_vel = 30

    velocity_pot = int(min(abs(a*cm_rel), max_vel))

    # ball is in left side of the drone and it's not too fast
    if cm_rel > stoping_limit and cm_rel < b * abs(real_vel) and real_vel < 0:   # If the velocity is positive we would like to stop
        velocity = -velocity_pot

    # ball is in right side of the drone and it's not too fast
    elif cm_rel < -stoping_limit and abs(cm_rel) < b * abs(real_vel) and real_vel > 0:    # If the velocity is negative we would like to stop
        velocity = velocity_pot

    else:
        velocity = 0
        
    return velocity


def lin_velocity_with_two_params(cm_rel, real_velocity, direction):
    #this function assumes the drone is looking at the cameras.
    MAX_VEL = 40
    A = 1.5
    B = 1
    VELOCITY_LIMIT = 20
    STOPPING_VEL = 40

    limit = B * real_velocity
    velocity_pot = int(min(A*(abs(cm_rel) - limit), MAX_VEL))
    
    if (abs(cm_rel) < limit and abs(real_velocity) > VELOCITY_LIMIT):
            velocity = -np.sign(real_velocity) * STOPPING_VEL
    
    else:
        velocity = -np.sign(cm_rel) * velocity_pot

    if direction == 'x':
        return int(velocity)
    elif direction == 'y':
        return int(velocity)
 

def lin_velocity_z(cm_rel):
    #this function assumes the drone is looking at the cameras.
    MAX_VEL = 40
    A = 2

    velocity_pot = int(min(A*(abs(cm_rel)), MAX_VEL))
    velocity = np.sign(cm_rel) * velocity_pot

    return int(velocity)


def lin_velocity_with_control(cm_rel, real_velocity):
    #this function assumes the drone is looking at the cameras.
    MAX_VEL = 80
    UPPER_LIMIT = 20
    LOWER_LIMIT = 5
    VELOCITY_LIMIT = 20
    STOPPING_VEL = 10
    A_UPPER = 1.5
    A_LOWER = 0.7

    velocity_pot_upper = int(min(A_UPPER*(abs(cm_rel) - LOWER_LIMIT), MAX_VEL))
    velocity_pot_lower = int(min(A_LOWER*(abs(cm_rel) - LOWER_LIMIT), MAX_VEL))
    

    # if drone is too fast, stop earlier
    if UPPER_LIMIT > abs(cm_rel) > LOWER_LIMIT and abs(real_velocity) > VELOCITY_LIMIT:
        velocity = 0

    # drone is not too fast, continue in original speed
    elif UPPER_LIMIT > abs(cm_rel) > LOWER_LIMIT:
        # velocity = np.sign(real_velocity)*velocity_pot_lower
        velocity = -np.sign(cm_rel)*velocity_pot_lower

    # if drone is too fast and close to the baloon, set negative velocity
    elif abs(cm_rel) < LOWER_LIMIT and abs(real_velocity) > VELOCITY_LIMIT:
        velocity = -np.sign(real_velocity)*STOPPING_VEL

    elif abs(cm_rel) > UPPER_LIMIT:
        velocity = np.sign(cm_rel)*velocity_pot_upper

    # if we got here, drone is close to the baloon with low speed
    else:
        velocity = 0

    return int(velocity)


def track_2d(image_3d: Image3D, tello: Tello, height:int=DRONE_DEFAULT_HEIGHT):
    x_cm_rel = image_3d.phys_x_balloon - image_3d.phys_x_drone
    #print("x_ball_cm: ", image_3d.phys_x_balloon, "x_drone_cm: ", image_3d.phys_x_drone)
    #print("x_cm_rel: ", x_cm_rel)

    y_cm_rel = image_3d.phys_y_balloon - image_3d.phys_y_drone
    #print("y_ball_cm: ", image_3d.phys_y_balloon, "y_drone_cm: ", image_3d.phys_y_drone)
    #print("y_cm_rel: ", y_cm_rel)

    z_cm_rel = height - image_3d.phys_z_drone

    left_right, for_back, up_down = 0, 0, 0
    if tello.send_rc_control:
        left_right = lin_velocity_with_two_params(x_cm_rel, image_3d.velocity_x_balloon, 'x')
        for_back = lin_velocity_with_two_params(y_cm_rel, image_3d.velocity_y_balloon, 'y')
        up_down = lin_velocity_z(z_cm_rel)
        tello.send_rc_control(left_right, for_back, up_down, 0)


def hit_ball(image: Image3D, tello: Tello):
    UPPER_LIMIT = 170
    LOWER_LIMIT = 30
    XY_LIMIT = 20
    A = 1.2
    
    x_rel = int(image.phys_x_balloon - image.phys_x_drone)
    y_rel = int(image.phys_y_balloon - image.phys_y_drone)
    z_rel = int(image.phys_z_balloon - image.phys_z_drone)

    if abs(x_rel) < XY_LIMIT and abs(y_rel) < XY_LIMIT and LOWER_LIMIT < z_rel < UPPER_LIMIT:
        if tello.send_rc_control:
            tello.go_xyz_speed(int(x_rel*A), int(y_rel*A), int(z_rel*A), 100)
            while not tello.send_rc_control:
                continue
            tello.go_xyz_speed(0, 0, -int(z_rel), 100)
        return True
    return False


def hit_ball_rc(image_3d: Image3D, tello: Tello, loop_status: Status):
    UPPER_LIMIT = 200
    LOWER_LIMIT = 0
    XY_LIMIT = 10
    Z_LIMIT = 15
    VEL_LIMIT = 5

    x_rel = int(image_3d.phys_x_balloon - image_3d.phys_x_drone)
    y_rel = int(image_3d.phys_y_balloon - image_3d.phys_y_drone)
    z_rel = int(loop_status.hit_height - image_3d.phys_z_drone)

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

            sleep(1)
            up_down = 0
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
        track_2d(image_3d, tello, int(loop_status.hit_height - 100))




def interactive_loop(image_3d: Image3D, colors: ColorBounds, loop_status: Status, tello: Tello) -> bool:
    key = cv2.waitKey(1) & 0xFF

    # the 'c' button reconnects to the drone
    if key == ord('c'):
        tello.connect()
        loop_status.reset()

    # the 'v' button is set as the detect color of balloon in the left cam
    if key == ord('v'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.ball_left.change(lower, upper)
        print("color bounds changed")

    # the 'n' button is set as the detect color of balloon in the right cam
    elif key == ord('n'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.ball_right.change(lower, upper)
        print("color bounds changed")

    # the 's' button is set as the detect color of drone in the left cam
    elif key == ord('s'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.drone_left.change(lower, upper)
        print("color bounds changed")

    # the 'f' button is set as the detect color of drone in the right cam
    elif key == ord('f'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.drone_right.change(lower, upper)
        print("color bounds changed")

    elif key == ord('t'):
        loop_status.takeoff()

    elif key == ord('y'):
        loop_status.start()

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        loop_status.stop_loop()
        return False

    # the 'l' button is set as the landing button
    elif key == ord('l'):
        loop_status.stop_loop()

    # the 'h' button is set as the hitting balloon method
    elif key == ord("h"):
        loop_status.hit_mode_on(image_3d.phys_z_balloon)

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
    tookoff = False
    continue_test = True

    loop_status = Status()
    old_images = [None]*10

    while(True):
        frame_counter = frame_counter+1
        # Capture the video frame by frame
        ret_left, image_left = vid_left.read()
        ret_right, image_right = vid_right.read()

        # if left.is_flipped:
        #     image_left = cv2.flip(image_left, 1)
        # if right.is_flipped:
        #     image_right = cv2.flip(image_right, 1)
        image_now = Image3D(image_left, image_right)
        text_balloon = None
        text_drone = None
    
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
        image_now.frame_left.show_image("left", text_balloon=text_balloon_coor, text_drone=text_drone_coor, text_color=(240,240,240))
        image_now.frame_right.show_image("right", text_balloon=text_balloon_vel, text_drone=text_drone_vel, text_color=(200,50,50))

        if loop_status.tookoff and not tookoff:
            tello.connect()
            print("battery = ", tello.get_battery(), "%")
            tello.takeoff()
            tookoff = True

        if loop_status.hit_mode():
            hit_ball_rc(image_now, tello, loop_status)
        elif loop_status.start_track:
            track_2d(image_now, tello)

        # if loop_status.hit_mode() and hit_ball(image_now, tello):
        #     loop_status.hit_mode_off()


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
