import numpy as np
import cv2
from color_bounds import ColorBounds
from image_3d import Image3D
from camera import Camera


def interactive_loop(key, image_3d, colors):
    continue_loop = True
    # the 'l' button is set as the detect color of balloon in the left cam
    if key == ord('l'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.ball_left.change(lower, upper)

    # the 'r' button is set as the detect color of balloon in the right cam
    elif key == ord('r'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.ball_right.change(lower, upper)

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        continue_loop = False

    return continue_loop


def capture_video(cameras_distance, left, right, method='parallel'):
    vid_left = left.vid
    vid_right = right.vid

    detect_left_time = 70
    detect_right_time = 140
    frame_counter = 0
    image_old = None
    colors = ColorBounds()

    while(True):
        frame_counter = frame_counter+1
        # Capture the video frame by frame
        ret_left, image_left = vid_left.read()
        ret_right, image_right = vid_right.read()

        image_left = cv2.flip(image_left, 1)
        image_right = cv2.flip(image_right, 1)
        image_now = Image3D(image_left, image_right)
        text_balloon = None
    
        # Process frames
        if frame_counter>1:
            image_now.detect_all(colors, image_old)
            balloon_exist, drone_exist = image_now.calculate_all_distances(left, right, cameras_distance, method=method)
            if not balloon_exist:
                image_now.phys_x_balloon, image_now.phys_y_balloon = image_old.phys_x_balloon, image_old.phys_y_balloon
            text_balloon = "(%.0f, %.0f)" % (image_now.phys_x_balloon, image_now.phys_y_balloon)
            if not drone_exist:
                image_now.phys_x_drone, image_now.phys_y_drone = image_old.phys_x_drone, image_old.phys_y_drone

        # Display the resulting frame
        image_now.frame_left.show_image("left, fps={}".format(left.fps))
        image_now.frame_right.show_image("right, fps={}".format(right.fps), text_balloon=text_balloon)

        image_old = image_now

        key = cv2.waitKey(1) & 0xFF
        if frame_counter == detect_left_time:
            print("detecting balloon color left")
            key = ord('l')
        if frame_counter == detect_right_time:
            print("detecting balloon color right")
            key = ord('r')    
        continue_loop = interactive_loop(key, image_now, colors)
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
    web = Camera(61, 0, True)
    phone = Camera(67, 1, True)
    distance = 82
    # Galaxy - FoV is 67 degrees
    # Lenovo - FoV is 61 degrees
    capture_video(distance, web, phone, method='parallel')
