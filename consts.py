from camera import Camera

FLOOR_HEIGHT = -114
DRONE_HEIGT = 14
DRONE_DEFAULT_HEIGHT = FLOOR_HEIGHT + DRONE_HEIGT + 90
DRONE_MIN_HEIGHT = FLOOR_HEIGHT + DRONE_HEIGT + 30

ORI_WEB = Camera(51.3, 51.3, 0, False)
ORI_PHONE = Camera(66.9, 66.9, 3, False)
EFRAT_WEB = Camera(61, 61, 2, False)

NIR_PHONE_NIR = Camera(67, 52, 0, False)
MAYA_PHONE_NIR = Camera(67, 55, 2, False)
EFRAT_PHONE_NIR = Camera(68, 77, 2, False)
C920_NIR_1 = Camera(56, 39, 4, False)
C920_NIR_2 = Camera(56, 39, 3, False)

C920_ORI_1 = Camera(56, 39, 7, False)
C920_ORI_2 = Camera(56, 39, 6, False)

COLORS_FILENAME = "color_bounds.txt"
BORDERS_FILENAME = "borders.txt"
