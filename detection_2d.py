import numpy as np
import cv2
from frame import Frame, ColorBounds

MAX_CHANGE = 5
NO_LOWER_BOUNDS = (0, 0, 0)
NO_UPPER_BOUNDS = (255,255, 255)


def interactive_loop(frame_web, frame_phone, ball_web, ball_phone):
    continue_loop = True
    key = cv2.waitKey(1) & 0xFF
    # the 'w' button is set as the detect color of balloon in the web cam
    if key == ord('w'):
        lower, upper = frame_web.detect_color()
        ball_web.change(lower, upper)

    # the 'p' button is set as the detect color of balloon in the phone cam
    elif key == ord('p'):
        lower, upper = frame_phone.detect_color()
        ball_phone.change(lower, upper)

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        continue_loop = False

    return continue_loop

# return updated distance from cam1 using pixels diffrence in cam2 
def update_distance(frame, old_frame, distance, angle):
    change = - (frame.x_balloon - old_frame.x_balloon) * pixels_to_cm(distance, frame.image.shape[1], angle) # we have to decide about directions beacuse it dependes on sign
    if (change <= MAX_CHANGE):
        return change
    return 0


def capture_video(distance_web, distance_phone, web_angle, phone_angle):
    vid_web = cv2.VideoCapture(0)
    vid_phone = cv2.VideoCapture(1)

    i = 0

    frame_phone_old = None
    frame_web_old = None
    ball_web = ColorBounds(NO_LOWER_BOUNDS, NO_UPPER_BOUNDS)
    ball_phone = ColorBounds(NO_LOWER_BOUNDS, NO_UPPER_BOUNDS)
    drone_web = ColorBounds(NO_LOWER_BOUNDS, NO_UPPER_BOUNDS)
    drone_web_phone = ColorBounds(NO_LOWER_BOUNDS, NO_UPPER_BOUNDS)
  
    while(True):  
        i = i+1
        # Capture the video frame by frame
        ret_web, image_web = vid_web.read()
        ret_phone, image_phone = vid_phone.read()

        frame_web = Frame(image_web)
        frame_phone = Frame(image_phone)
    
        # Process frames
        frame_web.detect_balloon(ball_web, frame_web_old.x_balloon, frame_web_old.y_balloon)
        frame_phone.detect_balloon(ball_phone, frame_phone_old.x_balloon, frame_phone_old.y_balloon)
        if i>1 and frame_web.x_balloon!=0 and frame_phone.x_balloon!=0:
            distance_web += update_distance(frame_phone, frame_phone_old, distance_phone, phone_angle)
            distance_phone += update_distance(frame_web, frame_web_old, distance_web, web_angle)

        if i%50 == 0:
            print("web distance is now " + str(distance_web))
            print("phone distance is now " + str(distance_phone))

        # Display the resulting frame
        frame_web.show_image("web")
        frame_phone.show_image("phone")

        frame_web_old = frame_web
        frame_phone_old = frame_phone

        continue_loop = interactive_loop(frame_web, frame_phone, ball_web, ball_phone)
        if not continue_loop:
            break
  
    # After the loop release the cap object
    vid_web.release()
    vid_phone.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


# return how much cm in one pixel. (i wrote this function sepperatly beacuse it may be useful later)
def pixels_to_cm(distance, num_pixels, theta):  # notice that theta is actually 0.5 of the total angle
    return distance * 2 * np.tan(theta) / num_pixels


if __name__ == "__main__":
    capture_video(60, 75, np.radians(33), np.radians(33))