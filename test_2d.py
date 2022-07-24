import numpy as np
import cv2
import PySimpleGUI as sg
from color_bounds import ColorBounds
from image_3d import Image3D
from loop_status import Status
from djitellopy import Tello
from camera import Camera
from gui import Gui

ORI_WEB = Camera(51.3, 0, False)
ORI_PHONE = Camera(66.9, 2, False)

def lin_velocity_with_acc(cm_rel, tello, direction):
    #this function assumes the drone is looking at the cameras.
    a = 1.5
    b = 1
    c = 0.5

    velocity_pot = int(min(abs(a*cm_rel), 30))
    real_vel = 0
    # if direction == 'x':
    #     real_vel = tello.get_speed_x()
    # elif direction == 'y':
    #     real_vel = tello.get_speed_y()

    # ball is in left side of the drone and it's not too fast
    if cm_rel > 5 and cm_rel > -b * real_vel:   # If the velocity is positive we would like to stop
        velocity = -velocity_pot

    # ball is in right side of the drone and it's not too fast
    elif cm_rel < -5 and cm_rel < -b * real_vel:    # If the velocity is negative we would like to stop
        velocity = velocity_pot

    else:
        velocity = 0
        
    return velocity



def track_2d(image_3d: Image3D, tello: Tello):
    x_cm_rel = image_3d.phys_x_balloon - image_3d.phys_x_drone
    print("x_ball_cm: ", image_3d.phys_x_balloon, "x_drone_cm: ", image_3d.phys_x_drone)
    print("x_cm_rel: ", x_cm_rel)

    y_cm_rel = image_3d.phys_y_balloon - image_3d.phys_y_drone
    print("y_ball_cm: ", image_3d.phys_y_balloon, "y_drone_cm: ", image_3d.phys_y_drone)
    print("y_cm_rel: ", y_cm_rel)

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
        left_right = lin_velocity_with_acc(x_cm_rel, tello, 'x')
        for_back = -1 * lin_velocity_with_acc(y_cm_rel, tello, 'y')
        tello.send_rc_control(left_right, for_back, up_down, 0)


def interactive_loop(frame_counter: int, image_3d: Image3D, colors: ColorBounds, gui: Gui, loop_status: Status, tello: Tello) -> bool:
    event, values = gui.window.read()

    if event == sg.WIN_CLOSED or event == 'Quit': # if user closes window
        loop_status.stop_loop()
        return False

    elif event == 'Ok':
        distance = int(values[1])
        gui.window['Ok'].update(button_color = ('white','blue'))  

    # the 'c' button reconnects to the drone
    if event == 'Connect':
        tello.connect()
        loop_status.reset()

    elif event == 'Balloon color (L)':
        lower, upper = image_3d.frame_left.detect_color()
        colors.ball_left.change(lower, upper)
        gui.window['Balloon color (L)'].update(button_color = ('white','blue'))

    elif event == 'Balloon color (R)':
        lower, upper = image_3d.frame_right.detect_color()
        colors.ball_right.change(lower, upper)
        gui.window['Balloon color (R)'].update(button_color = ('white','blue')) 

    elif event == 'Drone color (L)':
        lower, upper = image_3d.frame_left.detect_color()
        colors.drone_left.change(lower, upper)
        gui.window['Drone color (L)'].update(button_color = ('white','blue'))

    elif event == 'Drone color (R)':
        lower, upper = image_3d.frame_right.detect_color()
        colors.drone_right.change(lower, upper)
        gui.window['Drone color (R)'].update(button_color = ('white','blue'))

    elif event == 'Take Off':
        loop_status.takeoff()
        gui.window['Take Off'].update(button_color = ('white','blue'))

    elif event == 'Start Track':
        loop_status.start()
        gui.window['Start Track'].update(button_color = ('white','blue'))

    elif event == 'Land':
        loop_status.stop_loop()
        gui.window['Land'].update(button_color = ('white','blue'))

    return True


def capture_video(tello: Tello, cameras_distance, left: Camera, right: Camera, colors: ColorBounds, gui: Gui, method='parallel'):
    vid_left = cv2.VideoCapture(left.index)
    vid_right = cv2.VideoCapture(right.index)

    frame_counter = 0
    image_old = None
    tookoff = False
    continue_test = True

    loop_status = Status()

    while(True):
        frame_counter = frame_counter+1
        # Capture the video frame by frame
        ret_left, image_left = vid_left.read()
        ret_right, image_right = vid_right.read()

        image_left = cv2.flip(image_left, 1)
        image_right = cv2.flip(image_right, 1)
        image_now = Image3D(image_left, image_right)
        text_balloon = None
        text_drone = None
    
        # Process frames
        if frame_counter>1:
            image_now.detect_all(colors, image_old)
            balloon_exist, drone_exist = image_now.calculate_all_distances(left, right, cameras_distance, method=method)
            if not balloon_exist:
                image_now.phys_x_balloon, image_now.phys_y_balloon = image_old.phys_x_balloon, image_old.phys_y_balloon
            text_balloon = "(%.0f, %.0f)" % (image_now.phys_x_balloon, image_now.phys_y_balloon)
            if not drone_exist:
                image_now.phys_x_drone, image_now.phys_y_drone = image_old.phys_x_drone, image_old.phys_y_drone
            text_drone = "(%.0f, %.0f)" % (image_now.phys_x_drone, image_now.phys_y_drone)
    
        # Display the resulting frame
        image_now.frame_left.show_image("left", text_balloon=text_balloon, text_drone=text_drone)
        image_now.frame_right.show_image("right", text_balloon=text_balloon, text_drone=text_drone)

        if loop_status.tookoff and not tookoff:
            tello.takeoff()
            tookoff = True

        if loop_status.start_track:
            track_2d(image_now, tello)

        image_old = image_now
   
        continue_test = interactive_loop(frame_counter, image_now, colors, gui, loop_status, tello)
        if not loop_status.continue_loop:
            break
  
    tello.land()
    # After the loop release the cap object
    vid_left.release()
    vid_right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()
    gui.window.close()

    return continue_test, colors


# return how much cm in one pixel.
def pixels_to_cm(distance, num_pixels, fov_angle):  
    return distance * 2 * np.tan(fov_angle/2) * fov_angle / num_pixels


if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print("battery = ", tello.get_battery(), "%")

    colors = ColorBounds()
    gui = Gui()
    continue_test = True

    web = Camera(61, 0, True)
    phone = Camera(67, 1, True)
    distance = 59
    # Galaxy - FoV is 67 degrees
    # Lenovo - FoV is 61 degrees
    while continue_test:
        continue_test, colors = capture_video(tello, distance, ORI_PHONE, ORI_WEB, colors, gui, method='parallel')
