import numpy as np
from borders import Borders
import cv2

NUM_PIXELS = 512

def draw_xy_display(borders: Borders, x_balloon_phys, y_balloon_phys, x_drone_phys, y_drone_phys, x_pred_phys=None, y_pred_phys=None):
    xy_display = np.zeros((NUM_PIXELS, NUM_PIXELS, 3), np.uint8)

    x_low_limit = np.min(borders.coordinates[:,0])
    x_upper_limit = np.max(borders.coordinates[:,0])
    y_low_limit = np.min(borders.coordinates[:,1])
    y_upper_limit = np.max(borders.coordinates[:,1])

    if borders.set_borders:
        # calc coordinates in pixels
        x_balloon_pix = x_coor_to_pix(borders, x_balloon_phys)
        y_balloon_pix = y_coor_to_pix(borders, y_balloon_phys)
        x_drone_pix = x_coor_to_pix(borders, x_drone_phys)
        y_drone_pix = y_coor_to_pix(borders, y_drone_phys) 
        #x_pred_pix = int(((x_pred_phys - x_low_limit)/ (x_upper_limit - x_low_limit)) * NUM_PIXELS)
        #y_pred_pix = abs(NUM_PIXELS - int(((y_pred_phys - y_low_limit)/ (y_upper_limit - y_low_limit)) * NUM_PIXELS)) 

        print("coor balloon = " + "c(%.0f,%.0f)" % (x_balloon_pix, y_balloon_pix))
        print("coor drone = " + "c(%.0f,%.0f)" % (x_drone_pix, y_drone_pix))
        center_coordinates_balloon = (x_balloon_pix, y_balloon_pix)
        center_coordinates_drone = (x_drone_pix, y_drone_pix)  
        #center_coordinates_pred = (x_pred_phys, y_pred_phys)        

        xy_display = cv2.circle(xy_display, center_coordinates_drone, 15, (0,191,255), 3)
        xy_display = cv2.circle(xy_display, center_coordinates_balloon, 15, (255, 54, 89), 3)
        #xy_display = cv2.circle(xy_display, center_coordinates_pred, 15, (186,85,211), 3)
        
        xy_display = cv2.putText(xy_display, "drone", (x_drone_pix + 17, y_drone_pix),
                                       cv2.FONT_HERSHEY_DUPLEX, 0.5, (0,191,255), 2, cv2.LINE_AA)
        xy_display = cv2.putText(xy_display, "balloon", (x_balloon_pix + 17, y_balloon_pix),
                                       cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 54, 89), 2, cv2.LINE_AA)                               

        xy_display = cv2.line(xy_display, (x_coor_to_pix(borders, borders.coordinates[0][0]), y_coor_to_pix(borders,borders.coordinates[0][1])), (x_coor_to_pix(borders,borders.coordinates[1][0]), y_coor_to_pix(borders,borders.coordinates[1][1])), (240,0,240), thickness=2)
        xy_display = cv2.line(xy_display, (x_coor_to_pix(borders,borders.coordinates[1][0]), y_coor_to_pix(borders,borders.coordinates[1][1])), (x_coor_to_pix(borders,borders.coordinates[3][0]), y_coor_to_pix(borders,borders.coordinates[3][1])), (240,0,240), thickness=2)
        xy_display = cv2.line(xy_display, (x_coor_to_pix(borders,borders.coordinates[3][0]), y_coor_to_pix(borders,borders.coordinates[3][1])), (x_coor_to_pix(borders,borders.coordinates[2][0]), y_coor_to_pix(borders,borders.coordinates[2][1])), (240,0,240), thickness=2)
        xy_display = cv2.line(xy_display, (x_coor_to_pix(borders,borders.coordinates[2][0]), y_coor_to_pix(borders,borders.coordinates[2][1])), (x_coor_to_pix(borders,borders.coordinates[0][0]), y_coor_to_pix(borders,borders.coordinates[0][1])), (240,0,240), thickness=2)



    cv2.imshow('XY Display', xy_display)
    print("after imshow and set borders = " + str(borders.set_borders))


def x_coor_to_pix(borders: Borders, x):
    x_low_limit = np.min(borders.coordinates[:,0])
    x_upper_limit = np.max(borders.coordinates[:,0])

    return int(((x - x_low_limit)/ (x_upper_limit - x_low_limit)) * NUM_PIXELS)

def y_coor_to_pix(borders: Borders, y):
    y_low_limit = np.min(borders.coordinates[:,1])
    y_upper_limit = np.max(borders.coordinates[:,1])

    return abs(NUM_PIXELS - int(((y - y_low_limit)/ (y_upper_limit - y_low_limit)) * NUM_PIXELS))  