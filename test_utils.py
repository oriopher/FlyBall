import numpy as np
import cv2
from color_bounds import ColorBounds
from image_3d import Image3D
from loop_status import Status
from djitellopy import Tello
from camera import Camera
import utils

ORI_WEB = Camera(51.3, 0, False)
ORI_PHONE = Camera(66.9, 2, True)
NIR_PHONE = Camera(67, 1, True)
MAYA_WEB = Camera(61, 0, True)
EFRAT_WEB = Camera(61, 0, True)

COLORS_FILENAME = "color_bounds.txt"

def interactive_loop(image_3d: Image3D, colors: ColorBounds, loop_status: Status, tello: Tello) -> bool:
    key = cv2.waitKey(1) & 0xFF

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

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        loop_status.stop_loop()
        return False
        
    # the 'p' button is set as the save colors to file
    elif key == ord('p'):
        colors.write_colors(COLORS_FILENAME)

    # the 'k' button is set as the read colors from file
    elif key == ord('k'):
        colors.read_colors(COLORS_FILENAME)

    return True


def show_cams_with_circle(image3d: Image3D, cam : Camera):
    show_img = image3d.frame_left.image
    x_phys = image3d.phys_x_balloon
    y_phys = image3d.phys_y_balloon
    z_phys = image3d.phys_z_balloon

    radius = utils.phys_to_left_pix(x_phys + 10, y_phys, z_phys, show_img, cam) - utils.phys_to_left_pix(x_phys, y_phys, z_phys, show_img, cam)
    coordinates = utils.phys_to_left_pix(x_phys, y_phys, z_phys, show_img, cam)
    show_img = cv2.circle(show_img, coordinates, radius, (0, 0, 0), 3)
    cv2.imshow("left", show_img)
    cv2.imshow("right", image3d.frame_right.image)


def capture_video(tello: Tello, cameras_distance, left: Camera, right: Camera, colors: ColorBounds, method='parallel'):
    vid_left = cv2.VideoCapture(left.index)
    vid_right = cv2.VideoCapture(right.index)

    frame_counter = 0
    image_old = None
    continue_test = True

    loop_status = Status()
    old_images = [None]*10

    while(True):
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
        
        show_cams_with_circle(image_now, left)

        old_images[frame_counter % len(old_images)] = image_now
        image_old = image_now
   
        continue_test = interactive_loop(image_now, colors, loop_status, tello)
        if not loop_status.continue_loop:
            break

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

    left = MAYA_WEB
    right = NIR_PHONE

    distance = 75
    while continue_test:
        continue_test, colors = capture_video(tello, distance, left, right, colors, method='parallel')
