import numpy as np
import cv2
from color_bounds import ColorBounds
from image_3d import Image3D
from loop_status import Status
from djitellopy import Tello


class Camera:

    def __init__(self, fov: float, index: int, is_flipped=False) -> None:
        self.fov = np.radians(fov)
        self.index = index
        self.flip = -1 if is_flipped else 1
        self.is_flipped = is_flipped


def track_2d(image_3d: Image3D):
    pass


def interactive_loop(frame_counter: int, image_3d: Image3D, colors: ColorBounds, loop_status: Status) -> None:
    key = cv2.waitKey(1) & 0xFF

    detect_balloon_left_time = 150
    detect_balloon_right_time = 300
    detect_drone_left_time = 450
    detect_drone_right_time = 600
    takeoff_time = 1000
    start_track_time = 1200

    if frame_counter == detect_balloon_left_time:
        print("detecting balloon color left")
        key = ord('v')
    if frame_counter == detect_balloon_right_time:
        print("detecting balloon color right")
        key = ord('n') 
    if frame_counter == detect_drone_left_time:
        print("detecting drone color left")
        key = ord('s')
    if frame_counter == detect_drone_right_time:
        print("detecting drone color right")
        key = ord('f') 

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


def capture_video(cameras_distance: float, left: Camera, right: Camera, method='parallel') -> None:
    vid_left = cv2.VideoCapture(left.index)
    vid_right = cv2.VideoCapture(right.index)

    tello = Tello()
    tello.connect()
    print("battery = ", tello.get_battery(), "%")

    frame_counter = 0
    image_old = None
    tookoff = False

    colors = ColorBounds()
    loop_status = Status()

    while(True):
        frame_counter = frame_counter+1
        # Capture the video frame by frame
        ret_left, image_left = vid_left.read()
        ret_right, image_right = vid_right.read()

        image_left = cv2.flip(image_left, 1)
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
        image_now.frame_left.show_image("left")
        image_now.frame_right.show_image("right", text_balloon=text_balloon, text_drone=text_drone)

        if loop_status.tookoff and not tookoff:
            tello.takeoff()
            tookoff = True

        if loop_status.start_track:
            track_2d(image_now)

        image_old = image_now
   
        interactive_loop(frame_counter, image_now, colors, loop_status)
        if not loop_status.continue_loop:
            break
  
    tello.land()
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
    capture_video(distance, web, phone, method='parallel')
