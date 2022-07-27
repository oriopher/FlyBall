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

    elif key == ord('p'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.drone_1_left.change(lower, upper)

    elif key == ord('o'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.drone_1_right.change(lower, upper)            

    elif key == ord('i'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.drone_2_left.change(lower, upper)

    elif key == ord('u'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.drone_2_right.change(lower, upper)  

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        continue_loop = False

    return continue_loop


def capture_video(cameras_distance, left, right, method='parallel'):
    vid_left = cv2.VideoCapture(left.index)
    vid_right = cv2.VideoCapture(right.index)

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
            image_now.frame_left.detect_balloon(colors.ball_left, image_old.frame_left.x_balloon, image_old.frame_left.y_balloon)
            image_now.frame_right.detect_balloon(colors.ball_right, image_old.frame_right.x_balloon, image_old.frame_right.y_balloon)
            if image_now.frame_left.x_balloon!=0 and image_now.frame_right.x_balloon!=0:
                image_now.calculate_balloon_distance(left, right, cameras_distance, method=method)
                text_balloon = "(%.0f, %.0f)" % (image_now.phys_x_balloon, image_now.phys_y_balloon)

            image_now.frame_left.detect_drone_1(colors.drone_1_left, image_old.frame_left.x_drone_1, image_old.frame_left.y_drone_1)
            image_now.frame_right.detect_drone_1(colors.drone_1_right, image_old.frame_right.x_drone_1, image_old.frame_right.y_drone_1)

            image_now.frame_left.detect_drone_2(colors.drone_2_left, image_old.frame_left.x_drone_2, image_old.frame_left.y_drone_2)
            image_now.frame_right.detect_drone_2(colors.drone_2_right, image_old.frame_right.x_drone_2, image_old.frame_right.y_drone_2)

            image_now.calculate_mean_velocities(image_list)


        text_balloon = "(%.0f, %.0f)" % (image_now.velocity_x_balloon, image_now.velocity_y_balloon)
        text_drone_1 = "(%.0f, %.0f)" % (image_now.velocity_x_drone_1, image_now.velocity_y_drone_1)
        text_drone_2 = "(%.0f, %.0f)" % (image_now.velocity_x_drone_1, image_now.velocity_y_drone_2)
        
        # Display the resulting frame
        image_now.frame_left.show_image("left", text_balloon=text_balloon, text_drone_1=text_drone_1, text_drone_2=text_drone_2)
        image_now.frame_right.show_image("right", text_balloon=text_balloon)

        image_list[frame_counter % len(image_list)] = image_now
        image_old = image_now

        key = cv2.waitKey(1) & 0xFF
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
    web = Camera(61, 2, True)
    phone = Camera(67, 0, True)
    distance = 82
    # Galaxy - FoV is 67 degrees
    # Lenovo - FoV is 61 degrees
    capture_video(distance, phone, web, method='parallel')
