import numpy as np
import cv2
from color_bounds import ColorBounds
from image_3d import Image3D
from loop_status import Status
from djitellopy import Tello
from camera import Camera

ORI_WEB = Camera(51.3, 0, False)
ORI_PHONE = Camera(66.9, 0, True)
NIR_PHONE = Camera(67, 2, True)
MAYA_WEB = Camera(61, 1, True)

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


def lin_velocity_with_control(cm_rel, real_velocity):
    #this function assumes the drone is looking at the cameras.
    MAX_VEL = 55
    UPPER_LIMIT = 35
    LOWER_LIMIT = 10
    VELOCITY_LIMIT = 10
    STOPPING_VEL = 5
    A_UPPER = 1
    A_LOWER = 0.3
    B = -5

    velocity_pot_upper = int(min(abs(A_UPPER * cm_rel + B), MAX_VEL))
    velocity_pot_lower = int(min(abs(A_LOWER * cm_rel + B), MAX_VEL))
    

    # if drone is too fast, stop earlier
    if UPPER_LIMIT > abs(cm_rel) > LOWER_LIMIT and abs(real_velocity) > VELOCITY_LIMIT:
        velocity = 0

    # drone is not too fast, continue in original speed
    elif UPPER_LIMIT > abs(cm_rel) > LOWER_LIMIT:
        velocity = np.sign(real_velocity)*velocity_pot_lower

    # if drone is too fast and close to the baloon, set negative velocity
    elif abs(cm_rel) < LOWER_LIMIT and abs(real_velocity) > VELOCITY_LIMIT:
        velocity = -np.sign(real_velocity)*STOPPING_VEL

    elif cm_rel > UPPER_LIMIT:
        velocity = velocity_pot_upper

    elif cm_rel < -UPPER_LIMIT:
        velocity = -velocity_pot_upper

    # if we got here, drone is close to the baloon with low speed
    else:
        velocity = 0

    return int(velocity)


def track_2d(image_3d: Image3D, tello: Tello):
    x_cm_rel = image_3d.phys_x_balloon - image_3d.phys_x_drone
    #print("x_ball_cm: ", image_3d.phys_x_balloon, "x_drone_cm: ", image_3d.phys_x_drone)
    #print("x_cm_rel: ", x_cm_rel)

    y_cm_rel = image_3d.phys_y_balloon - image_3d.phys_y_drone
    #print("y_ball_cm: ", image_3d.phys_y_balloon, "y_drone_cm: ", image_3d.phys_y_drone)
    #print("y_cm_rel: ", y_cm_rel)

    left_right, for_back, up_down = 0, 0, 0
    if tello.send_rc_control:
        # if 20 <= x_cm_rel <= 30:
        #     tello.send_rc_control(left_right, for_back, up_down, 0)
        #     # tello.move_right(int(x_cm_rel))
        # elif -30 <= x_cm_rel <= -20:
        #     tello.send_rc_control(left_right, for_back, up_down, 0)
        #     # tello.move_left(int(x_cm_rel))
        # elif 20 <= y_cm_rel <= 30:
        #     tello.send_rc_control(left_right, for_back, up_down, 0)
        #     # tello.move_forward(int(y_cm_rel))
        # elif -30 <= y_cm_rel <= -20:
        #     tello.send_rc_control(left_right, for_back, up_down, 0)
        #     # tello.move_back(int(y_cm_rel))
        # else:
        left_right = -lin_velocity_with_control(x_cm_rel, image_3d.velocity_x_balloon)
        for_back = -lin_velocity_with_control(y_cm_rel, image_3d.velocity_x_balloon)
        tello.send_rc_control(left_right, for_back, up_down, 0)


def interactive_loop(frame_counter: int, image_3d: Image3D, colors: ColorBounds, loop_status: Status, tello: Tello) -> bool:
    key = cv2.waitKey(1) & 0xFF

    # detect_balloon_left_time = 200
    # detect_balloon_right_time = 400
    # detect_drone_left_time = 600
    # detect_drone_right_time = 800
    # takeoff_time = 1000
    # start_track_time = 1200

    # if frame_counter == detect_balloon_left_time:
    #     print("detecting balloon color left")
    #     key = ord('v')
    # if frame_counter == detect_balloon_right_time:
    #     print("detecting balloon color right")
    #     key = ord('n') 
    # if frame_counter == detect_drone_left_time:
    #     print("detecting drone color left")
    #     key = ord('s')
    # if frame_counter == detect_drone_right_time:
    #     print("detecting drone color right")
    #     key = ord('f') 

    # the 'c' button reconnects to the drone
    if key == ord('c'):
        tello.connect()
        loop_status.reset()

    # the 'v' button is set as the detect color of balloon in the left cam
    if key == ord('v'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.ball_left.change(lower, upper)

    # the 'n' button is set as the detect color of balloon in the right cam
    elif key == ord('n'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.ball_right.change(lower, upper)

    # the 's' button is set as the detect color of drone in the left cam
    elif key == ord('s'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.drone_left.change(lower, upper)

    # the 'f' button is set as the detect color of drone in the right cam
    elif key == ord('f'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.drone_right.change(lower, upper)

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

        if left.is_flipped:
            image_left = cv2.flip(image_left, 1)
        if right.is_flipped:
            image_right = cv2.flip(image_right, 1)
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
        
        text_balloon_coor = "c(%.0f, %.0f)" % (image_now.phys_x_balloon, image_now.phys_y_balloon)
        text_drone_coor = "c(%.0f, %.0f)" % (image_now.phys_x_drone, image_now.phys_y_drone)
        text_balloon_vel = "v(%.0f, %.0f)" % (image_now.velocity_x_balloon, image_now.velocity_y_balloon)
        text_drone_vel = "v(%.0f, %.0f)" % (image_now.velocity_x_drone, image_now.velocity_y_drone)
    
        # Display the resulting frame
        image_now.frame_left.show_image("left", text_balloon=text_balloon_coor, text_drone=text_drone_coor, text_color=(240,240,240))
        image_now.frame_right.show_image("right", text_balloon=text_balloon_vel, text_drone=text_drone_vel, text_color=(200,50,50))

        if loop_status.tookoff and not tookoff:
            tello.connect()
            print("battery = ", tello.get_battery(), "%")
            tello.takeoff()
            tookoff = True

        if loop_status.start_track:
            track_2d(image_now, tello)

        old_images[frame_counter % len(old_images)] = image_now
        image_old = image_now
   
        continue_test = interactive_loop(frame_counter, image_now, colors, loop_status, tello)
        if not loop_status.continue_loop:
            break
  
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

    distance = 48
    # Galaxy - FoV is 67 degrees
    # Lenovo - FoV is 61 degrees
    while continue_test:
        continue_test, colors = capture_video(tello, distance, MAYA_WEB, ORI_PHONE, colors, method='parallel')
