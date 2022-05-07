import numpy as np
import cv2
from frame import Frame, ColorBounds, Image3D

MAX_CHANGE = 5
NO_LOWER_BOUNDS = (0, 0, 0)
NO_UPPER_BOUNDS = (255,255, 255)


def interactive_loop(key, image_3d, colors):
    continue_loop = True
    # the 'w' button is set as the detect color of balloon in the web cam
    if key == ord('w'):
        lower, upper = image_3d.frame_web.detect_color()
        colors.ball_web.change(lower, upper)

    # the 'p' button is set as the detect color of balloon in the phone cam
    elif key == ord('p'):
        lower, upper = image_3d.frame_phone.detect_color()
        colors.ball_phone.change(lower, upper)

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        continue_loop = False

    return continue_loop


def capture_video(cameras_distance, web_fov, phone_fov):
    vid_web = cv2.VideoCapture(0)
    vid_phone = cv2.VideoCapture(1)

    detect_web_time = 150
    detect_phone_time = 300

    frame_counter = 0

    image_old = None

    colors = ColorBounds()
  
    while(True):  
        frame_counter = frame_counter+1
        # Capture the video frame by frame
        ret_web, image_web = vid_web.read()
        ret_phone, image_phone = vid_phone.read()

        image_web = cv2.flip(image_web, 1)
        image_now = Image3D(image_web, image_phone)
        text_balloon = None
    
        # Process frames
        if frame_counter>1:
            image_now.frame_web.detect_balloon(colors.ball_web, image_old.frame_web.x_balloon, image_old.frame_web.y_balloon)
            image_now.frame_phone.detect_balloon(colors.ball_phone, image_old.frame_phone.x_balloon, image_old.frame_phone.y_balloon)
            if image_now.frame_web.x_balloon!=0 and image_now.frame_phone.x_balloon!=0:
                image_now.calculate_distance(web_fov, phone_fov, cameras_distance)
                text_balloon = "(%.0f, %.0f)" % (image_now.phys_x, image_now.phys_y)

        # Display the resulting frame
        image_now.frame_web.show_image("web")
        image_now.frame_phone.show_image("phone", text_balloon=text_balloon)

        image_old = image_now

        key = cv2.waitKey(1) & 0xFF
        if frame_counter == detect_web_time:
            print("detecting ballon color web")
            key = ord('w')
        if frame_counter == detect_phone_time:
            print("detecting ballon color phone")
            key = ord('p')    
        continue_loop = interactive_loop(key, image_now, colors)
        if not continue_loop:
            break
  
    # After the loop release the cap object
    vid_web.release()
    vid_phone.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


# return how much cm in one pixel.
def pixels_to_cm(distance, num_pixels, fov_angle):  
    return distance * 2 * np.tan(fov_angle/2) / num_pixels


if __name__ == "__main__":
    # Galaxy - FoV is 67 degrees
    # Lenovo - FoV is 61 degrees
    capture_video(119, np.radians(61), np.radians(67))