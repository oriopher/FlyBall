import numpy as np
import cv2
from color_bounds import ColorBounds
from image_3d import Image3D
from loop_status import Status
from camera import Camera
from prediction import NumericBallPredictor
from borders import Borders
from common import *


def interactive_loop(image_3d: Image3D, colors: ColorBounds, borders : Borders, loop_status: Status, left_cam : Camera) -> bool:
    key = cv2.waitKey(1) & 0xFF

    # the 'v' button is set as the detect color of recognizable_object in the left_cam cam
    if key == ord('v'):
        lower, upper = image_3d.frame_left.detect_color()
        colors.ball_left.change(lower, upper)
        print("color bounds changed")

    # the 'n' button is set as the detect color of recognizable_object in the right_cam cam
    elif key == ord('n'):
        lower, upper = image_3d.frame_right.detect_color()
        colors.ball_right.change(lower, upper)
        print("color bounds changed")

    # the 'q' button is set as the quitting button
    elif key == ord('q'):
        loop_status.stop_loop()
        return False

    # the 'p' button is set as the save text_colors to file
    elif key == ord('p'):
        colors.write_colors(COLORS_FILENAME)

    # the 'k' button is set as the read text_colors from file
    elif key == ord('k'):
        colors.read_colors(COLORS_FILENAME)

    # the 'z' button is set as the start predictions
    elif key == ord('z'):
        loop_status.ready_to_test()

    # the 'x' button is set as the test predictions
    elif key == ord('x'):
        loop_status.test_predictions()

    # the 'c' button is set as the test predictions
    elif key == ord('c'):
        loop_status.stop_predictions()

    # the 'j' button is set as the saving the borders. can save 4 coordinates
    elif key == ord('j'):
        borders.set_image(image_3d, left_cam)
        print("saved the %.0f coordinate: (%.0f,%.0f,%.0f)" % (borders.index, image_3d.phys_x_balloon, image_3d.phys_y_balloon, image_3d.phys_z_balloon))
        if borders.index == 4:
            borders.save_borders(BORDERS_FILENAME)

    # the 'r' button is set as the read text_colors from file
    elif key == ord('r'):
        borders.load_borders(BORDERS_FILENAME)
        print("middle is ({0:.3f},{1:.3f})".format(borders.x_middle, borders.y_middle))

    return True


def predict(image_3d : Image3D):
    NUM_PREDICTIONS = 61
    LATEST_TIME = 2 # in seconds, relative to image.time.

    # times = np.linspace(0, LATEST_TIME, NUM_PREDICTIONS)
    # predictions = np.zeros((NUM_PREDICTIONS, 4)) # every row is a prediction with (time, x, y, z)
    predictor = NumericBallPredictor(image_3d)
    # for i in range(len(predictions)):
    #     predictions[i][0] = predictor.time + datetime.timedelta(seconds = times[i])
    #     predictions[i][1], predictions[i][2], predictions[i][3] = predictor.get_prediction(times[i])
    
    # return predictions

    return predictor


def print_prediction_test(predictions, results):
    diff = results - predictions
    printable = np.concatenate((predictions, results, diff), axis = 1)
    title = np.array(["pred time", "x", "y", "z", "res time", "x", "y", "z", "diff time", "diff x", "diff y", "diff z"])
    print(np.concatenate((title, printable)))


def save_prediction(prediction_table, index, diff_time, real_coord, pred_coord):
    prediction_table[index][0] = diff_time
    prediction_table[index][1] = real_coord[0]
    prediction_table[index][2] = real_coord[1]
    prediction_table[index][3] = real_coord[2]
    prediction_table[index][4] = pred_coord[0]
    prediction_table[index][5] = pred_coord[1]
    prediction_table[index][6] = pred_coord[2]


def capture_video( cameras_distance, left: Camera, right: Camera, method='parallel'):
    vid_left = cv2.VideoCapture(left.index)
    vid_right = cv2.VideoCapture(right.index)

    frame_counter = 0
    image_old = None
    continue_test = True
    predictor = None

    loop_status = Status()
    colors = ColorBounds()
    borders = Borders()
    old_images = [None]*10
    prediction_table = np.zeros((40,7))

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
                image_now.phys_x_drone_1, image_now.phys_y_drone_1 = image_old.phys_x_drone_1, image_old.phys_y_drone_1

            image_now.calculate_mean_velocities(old_images)

        if loop_status.get_predict_stat() == 2: # test prediction
            diff_time = image_now.time - predictor.time
            diff_time = diff_time.total_seconds()
            x_pred, y_pred, z_pred = predictor.get_prediction(diff_time)
            x_real, y_real, z_real = image_now.phys_x_balloon, image_now.phys_y_balloon, image_now.phys_z_balloon
            prediction_index = frame_counter - prediction_start
            if prediction_index < len(prediction_table):
                save_prediction(prediction_table, prediction_index, diff_time, (x_real, y_real, z_real), (x_pred, y_pred, z_pred))
      
        text_balloon_coor = "c(%.0f,%.0f,%.0f)" % (image_now.phys_x_balloon, image_now.phys_y_balloon, image_now.phys_z_balloon)
        text_balloon_vel = "v(%.0f,%.0f,%.0f)" % (image_now.velocity_x_balloon, image_now.velocity_y_balloon, image_now.velocity_z_balloon)
    
        # Display the resulting frame
        left_show_img = image_now.frame_left.image_to_show(text_balloon=text_balloon_coor, text_color=(240,240,240))
        left_show_img = borders.draw_borders(left_show_img, image_now, color_in=(0, 240, 0), color_out=(0, 0, 240))
        if loop_status.get_predict_stat() == 2: 
            left_show_img = image_with_circle(left, left_show_img, (x_pred, y_pred, z_pred), rad_phys=11.3)
        cv2.imshow("left_cam", left_show_img)
        image_now.frame_right.show_image("right_cam", text_balloon=text_balloon_vel, text_color=(200,50,50))

        if loop_status.get_predict_stat() == 4 and borders.in_borders(image_now):
            loop_status.start_predictions()

        if loop_status.get_predict_stat() == 1: # start prediction
            predictor = NumericBallPredictor(image_now)
            loop_status.test_predictions()
            prediction_start = frame_counter + 1

        old_images[frame_counter % len(old_images)] = image_now
        image_old = image_now
   
        continue_test = interactive_loop(image_now, colors, borders, loop_status, left)
        if not loop_status.continue_loop:
            break


    # After the loop release the cap object
    vid_left.release()
    vid_right.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

    shape = image_left.shape
    return continue_test, prediction_table, shape


# return how much cm in one pixel.
def pixels_to_cm(distance, num_pixels, fov_angle):  
    return distance * 2 * np.tan(fov_angle/2) * fov_angle / num_pixels


if __name__ == "__main__":
    continue_test = True

    left = MAYA_PHONE_NIR
    right = EFRAT_PHONE_NIR

    distance = 69
    while continue_test:
        continue_test, prediction_table, shape = capture_video(distance, left, right, method='parallel')

    for prediction in prediction_table:
        print("time difference is %.4f sec" % prediction[0])
        print("prediction coords: (%.0f,%.0f,%.0f)" % (prediction[4], prediction[5], prediction[6]))
        print("real coords: (%.0f,%.0f,%.0f)" % (prediction[1], prediction[2], prediction[3]))
        print("diff is (%.0f,%.0f,%.0f)" % (prediction[4] - prediction[1], prediction[5] - prediction[2], prediction[6]-prediction[3]))
        print("------------------------------")


    pred_image = np.zeros((shape[0],shape[1],shape[2]), np.uint8)
    for prediction in prediction_table[::3]:
        pred_image = image_with_circle(left, pred_image, prediction[1], prediction[2], prediction[3], color=(240,50,240), thickness=1)
        pred_image = image_with_circle(left, pred_image, prediction[4], prediction[5], prediction[6], color=(50,240,240), thickness=1)

    cv2.imshow("prediction", pred_image)
    cv2.waitKey(0)