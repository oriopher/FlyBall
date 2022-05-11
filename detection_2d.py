import numpy as np
import cv2
from frame import Frame, ColorBounds, Image3D

MAX_CHANGE = 5
NO_LOWER_BOUNDS = (0, 0, 0)
NO_UPPER_BOUNDS = (255,255, 255)


class Camera:

    def __init__(self, fov, index, is_flipped=False):
        self.fov = np.radians(fov)
        self.index = index
        self.flip = -1 if is_flipped else 1


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


def capture_video(cameras_distance, left, right):
    vid_left = cv2.VideoCapture(left.index)
    vid_right = cv2.VideoCapture(right.index)

    detect_left_time = 150
    detect_right_time = 300

    frame_counter = 0

    image_old = None

    colors = ColorBounds()
    while(True):
        frame_counter = frame_counter+1
        # Capture the video frame by frame
        ret_left, image_left = vid_left.read()
        ret_right, image_right = vid_right.read()

        image_left = cv2.flip(image_left, 1)
        image_now = Image3D(image_left, image_right)
        text_balloon = None
    
        # Process frames
        if frame_counter>1:
            image_now.frame_left.detect_balloon(colors.ball_left, image_old.frame_left.x_balloon, image_old.frame_left.y_balloon)
            image_now.frame_right.detect_balloon(colors.ball_right, image_old.frame_right.x_balloon, image_old.frame_right.y_balloon)
            if image_now.frame_left.x_balloon!=0 and image_now.frame_right.x_balloon!=0:
                image_now.calculate_distance(left, right, cameras_distance)
                text_balloon = "(%.0f, %.0f)" % (image_now.phys_x, image_now.phys_y)

        # Display the resulting frame
        image_now.frame_left.show_image("left")
        image_now.frame_right.show_image("right", text_balloon=text_balloon)

        image_old = image_now

        key = cv2.waitKey(1) & 0xFF
        if frame_counter == detect_left_time:
            print("detecting ballon color left")
            key = ord('w')
        if frame_counter == detect_right_time:
            print("detecting ballon color right")
            key = ord('p')    
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
    web = Camera(51.3, 0, True)
    phone = Camera(66.9, 2, False)
    distance = 46.5
    # Galaxy - FoV is 67 degrees
    # Lenovo - FoV is 61 degrees
    capture_video(distance, web, phone)
