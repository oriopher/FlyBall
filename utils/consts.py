from images.camera import Camera

FLOOR_HEIGHT = -114
DRONE_HEIGHT = 14
DRONE_DEFAULT_HEIGHT = FLOOR_HEIGHT + DRONE_HEIGHT + 60
DRONE_MIN_HEIGHT = FLOOR_HEIGHT + DRONE_HEIGHT + 30

C920_NIR_1 = Camera(56, 39, 4, False)
C920_NIR_2 = Camera(56, 39, 3, False)

C920_ORI_1 = Camera(56, 39, 7, False)
C920_ORI_2 = Camera(56, 39, 6, False)

COLORS_FILENAME = "color_bounds.txt"
BORDERS_FILENAME = "../borders.txt"
