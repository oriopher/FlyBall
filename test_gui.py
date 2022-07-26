from datetime import datetime, timedelta
import numpy as np
import cv2
import PySimpleGUI as sg
from borders import Borders
from gui import Gui
from color_bounds import ColorBounds
from image_3d import Image3D
from loop_status import Status
from djitellopy import Tello
from camera import Camera
from velocity_pot import lin_velocity_with_two_params, track_balloon, seek_middle

ORI_WEB = Camera(51.3, 0, False)
ORI_PHONE = Camera(66.9, 3, False)
NIR_PHONE = Camera(67, 4, False)
MAYA_WEB = Camera(61, 0, True)
EFRAT_WEB = Camera(61, 2, False)
EFRAT_PHONE = Camera(64, 3, False)

NIR_PHONE_NIR = Camera(67, 3, False)
EFRAT_PHONE_NIR = Camera(77, 2, False)

COLORS_FILENAME = "color_bounds.txt"

FLOOR_HEIGHT = -80
DRONE_DEFAULT_HEIGHT = FLOOR_HEIGHT + 50



def interactive_loop(image_3d: Image3D, colors: ColorBounds, borders: Borders, gui: Gui, loop_status: Status, left_cam: Camera) -> bool:
    event, values = gui.window.read()
    key = cv2.waitKey(1) & 0xFF
    str_colors_changed = "color bounds changed"

    # the 'q' button is set as the quitting button or if user closes window
    if event == sg.WIN_CLOSED or event == 'Quit' or key == ord('q'):
        loop_status.stop_loop()
        return False

    # if user presses ENTER it updates distance
    elif event == 'Ok':
        distance = int(values[4])
        gui.window['Ok'].update(button_color = ('white','blue'))  


    # the 'v' button is set as the detect color of balloon in the left cam
    elif event == 'Balloon color (L)' or key == ord('v'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.ball_left.change(lower, upper)
        print(str_colors_changed)
        gui.window['Balloon color (L)'].update(button_color = ('white','blue'))

    # the 'n' button is set as the detect color of balloon in the right cam
    elif event == 'Balloon color (R)' or key == ord('n'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.ball_right.change(lower, upper)
        print(str_colors_changed)
        gui.window['Balloon color (R)'].update(button_color = ('white','blue')) 

    # the 's' button is set as the detect color of drone in the left cam
    elif event == 'Drone color (L)' or key == ord('s'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.drone_left.change(lower, upper)
        print(str_colors_changed)
        gui.window['Drone color (L)'].update(button_color = ('white','blue'))

    # the 'f' button is set as the detect color of drone in the right cam
    elif event == 'Drone color (R)' or key == ord('f'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.drone_right.change(lower, upper)
        print(str_colors_changed)
        gui.window['Drone color (R)'].update(button_color = ('white','blue'))

    # the 'y' button is set as start tracking balloon
    elif event == 'Start Track' or key == ord('y'):
        loop_status.start_track()
        gui.window['Start Track'].update(button_color = ('white','blue'))

    # the 'l' button is set as the landing button
    elif event == 'Land' or key == ord('l'):
        loop_status.stop_loop()
        gui.window['Land'].update(button_color = ('white','blue'))

    # the 'h' button is set as the hitting balloon method
    elif event == 'Hit' or key == ord('h'):
        coords = (image_3d.phys_x_balloon, image_3d.phys_y_balloon, image_3d.phys_z_balloon)
        loop_status.hit_mode_on(coords)


    # the 'p' button is set as the save colors to file
    elif event == 'Save Colors' or key == ord('p'):
        colors.write_colors(COLORS_FILENAME)
        gui.window['Save Colors'].update(button_color = ('white','blue'))

    # the 'k' button is set as the read colors from file
    elif event == 'Load Colors' or key == ord('k'):
        colors.read_colors(COLORS_FILENAME)
        gui.window['Load Colors'].update(button_color = ('white','blue'))

    # the 'j' button is set as the saving the borders. can save 4 coordinates
    elif event == '-SetBorders-' or key == ord('j'):
        borders.set_image(image_3d, left_cam)
        gui.window['-SetBorders-'].update('Set Borders('+ str(borders.index) + ')')
        print("saved the %.0f coordinate: (%.0f,%.0f,%.0f)" % (borders.index, image_3d.phys_x_balloon, image_3d.phys_y_balloon, image_3d.phys_z_balloon))
        if borders.index == 4:
            borders.write_borders('borders.txt')
            gui.window['-SetBorders-'].update(button_color = ('white','blue'))    

    # the 'r' button is set as the read colors from file
    elif event == 'Load Borders' or key == ord('r'):
        borders.read_borders('borders.txt')
        print("middle is ({0:.3f},{1:.3f})".format(borders.x_middle, borders.y_middle))
        gui.window['Load Borders'].update(button_color = ('white','blue'))
        gui.window['-MIDDLE-'].update("Middle = ({0:.3f},{1:.3f})".format(borders.x_middle, borders.y_middle))

    return True


def capture_video(cameras_distance, left: Camera, right: Camera, colors: ColorBounds, borders: Borders, gui: Gui, method='parallel'):
    vid_left = cv2.VideoCapture(left.index)
    vid_right = cv2.VideoCapture(right.index)

    frame_counter = 0
    image_old = None
    continue_test = True

    loop_status = Status()
    old_images = [None]*15

    while(True):
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
        left_img = image_now.frame_left.image_to_show("left", text_balloon=text_balloon_coor, text_drone=text_drone_coor, text_color=(150,250,200))
        color = (0, 240, 0)
        if not borders.balloon_in_borders(image_now):
            color = (0, 0, 240)
        left_img = borders.draw_borders(left_img, color)
        cv2.imshow("left", left_img)
        image_now.frame_right.show_image("right", text_balloon=text_balloon_vel, text_drone=text_drone_vel, text_color=(240,150,240))

        # balloon is out of borders. drone is seeking the middle until the balloon is back
        if borders.set_borders and loop_status.first_seek and (not borders.balloon_in_borders(image_now) or not loop_status.start):
            print("seek middle")
            loop_status.stop_track()
            #seek_middle(image_now, tello, borders)

        # balloon returned to the play area, we can continue to play
        #if borders.in_borders(image_now) and not loop_status.start_track:
        #   loop_status.out_of_borders = False
        #   loop_status.start_track = True

    
        elif loop_status.hit_time and datetime.now() - timedelta(seconds = 1) > loop_status.hit_time:
            continue


        old_images[frame_counter % len(old_images)] = image_now
        image_old = image_now
    
        continue_test = interactive_loop(image_now, colors, borders, gui, loop_status, left)
        if not loop_status.continue_loop:
            break
    

    # After the loop release the cap object
    vid_left.release()
    vid_right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

    return continue_test, colors


if __name__ == "__main__":

    colors = ColorBounds()
    borders = Borders()
    gui = Gui()
    continue_test = True

    left = NIR_PHONE
    right = ORI_PHONE

    distance = 77
    while continue_test:
        continue_test, colors = capture_video(distance, left, right, colors, borders, gui, method='parallel')
