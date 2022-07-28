from matplotlib.pyplot import draw
import numpy as np
from borders import Borders
import cv2
from loop_status import Status
from xy_display import draw_xy_display
from color_bounds import ColorBounds
from image_3d import Image3D
from camera import Camera
from utils import image_with_circle

BORDERS_FILENAME = "borders.txt"
COLORS_FILENAME = "color_bounds.txt"

def interactive_loop(key, image_3d, colors, left_cam):
    continue_loop = True
    str_colors_changed = "color bounds changed"

    # the 'l' button is set as the detect color of balloon in the left cam
    if key == ord('l'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.ball_left.change(lower, upper)

    # the 'r' button is set as the detect color of balloon in the right cam
    elif key == ord('r'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.ball_right.change(lower, upper)

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
    
    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        continue_loop = False

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
    elif key == ord('t'):
        borders.read_borders(BORDERS_FILENAME)
        print("middle is ({0:.3f},{1:.3f})".format(borders.x_middle, borders.y_middle))
    

    return continue_loop


def capture_video(cameras_distance, left, right, borders, method='parallel'):
    vid_left = cv2.VideoCapture(left.index)
    vid_right = cv2.VideoCapture(right.index)
    loop_status = Status()
    frame_counter = 0
    image_old = None

    image_list = [None] * 10

    colors = ColorBounds()

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
    
        # Process frames

        if frame_counter>len(image_list):
            image_now.detect_all(colors, image_old)
            balloon_exist, drone_exist = image_now.calculate_all_distances(left, right, cameras_distance, method=method)
            if not balloon_exist:
                image_now.phys_x_balloon, image_now.phys_y_balloon, image_now.phys_z_balloon = image_old.get_phys_balloon(0), image_old.get_phys_balloon(1), image_old.get_phys_balloon(2)
            if not drone_exist:
                image_now.phys_x_drone, image_now.phys_y_drone, image_now.phys_z_drone = image_old.get_phys_drone(0), image_old.get_phys_drone(1), image_old.get_phys_drone(2)

            image_now.calculate_mean_velocities(image_list)
            text_balloon = "(%.0f, %.0f)" % (image_now.velocity_x_balloon, image_now.velocity_y_balloon)
        
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

        draw_xy_display(borders, image_now.phys_x_balloon, image_now.phys_y_balloon, image_now.phys_x_drone, image_now.phys_y_drone)

        image_list[frame_counter % len(image_list)] = image_now
        image_old = image_now

        key = cv2.waitKey(1) & 0xFF
        continue_loop = interactive_loop(key, image_now, colors, left)
        if not continue_loop:
            break
  
    # After the loop release the cap object
    vid_left.release()
    vid_right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


# return how much cm in one pixel.
def pixels_to_cm(distance, num_pixels, fov_angle):  
    return distance * 2 * np.tan(fov_angle/2) * fov_angle / num_pixels


if __name__ == "__main__":
    web = Camera(61, 2, 2, True)
    phone = Camera(67, 2, 0, True)
    borders = Borders()
    distance = 61
    # Galaxy - FoV is 67 degrees
    # Lenovo - FoV is 61 degrees
    capture_video(distance, phone, web, borders, method='parallel')
