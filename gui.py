from utils.image_utils import display_frames
import PySimpleGUI as sg
import cv2


class Gui:
    """
    A class for the Graphic User Interface.
    """
    WINDOW_CLOSED = sg.WIN_CLOSED
    colors_col = sg.Column([[sg.Button('Drone 1 color (L)'), sg.Button('Drone 1 color (R)')],
                            [sg.Button('Drone 2 color (L)'), sg.Button('Drone 2 color (R)')],
                            [sg.Button('Balloon color (L)'), sg.Button('Balloon color (R)')]],
                           element_justification='c')

    read_col = sg.Column([[sg.Button('Save Colors'), sg.Button('Set Borders')],
                          [sg.Button('Load Colors'), sg.Button('Load Borders')]], element_justification='c')

    distance_col = sg.Column([[sg.Text('Distance:'), sg.InputText(size=(4, 1), key='distance'),
                               sg.Button('received_input', visible=False, bind_return_key=True)],
                              [sg.Text('Home 1 = ', key='home_1'), sg.InputText(size=(8, 1), key='set_home_1')],
                              [sg.Text('Home 2 = ', key='home_2'), sg.InputText(size=(8, 1), key='set_home_2')]],
                             element_justification='c')

    play_func = sg.Column([[sg.Button('Take Off'), sg.Button('Go Home'), sg.Button('Start Play'), sg.Button('Quit')]],
                          element_justification='c')

    left_camera_col = sg.Column([[sg.Text('Left Camera:'), sg.Push()],
                                 [sg.Text('Index:'), sg.InputText(size=(4, 1), key='left_index'),
                                  sg.Checkbox('Flip', key='flip_left')]], element_justification='c')

    right_camera_col = sg.Column([[sg.Text('Right Camera:'), sg.Push()],
                                  [sg.Text('Index:'), sg.InputText(size=(4, 1), key='right_index'),
                                   sg.Checkbox('Flip', key='flip_right')]], element_justification='c')

    color_bounds_col1 = sg.Column([[sg.Text('THRESHOLD', font=("Helvetica", 8)),
                                    sg.Slider((1, 600), 120, orientation='h', key='SLIDER_T', enable_events=True,
                                              size=(11, 11))],
                                   [sg.Text('H RANGE     ', font=("Helvetica", 8)),
                                    sg.Slider((0, 30), 15, orientation='h', key='SLIDER_H', enable_events=True,
                                              size=(11, 11))]], element_justification='c')

    color_bounds_col2 = sg.Column([[sg.Text('S RANGE     ', font=("Helvetica", 8)),
                                    sg.Slider((20, 40), 30, orientation='h', key='SLIDER_S', enable_events=True,
                                              size=(11, 11))],
                                   [sg.Text('H RANGE     ', font=("Helvetica", 8)),
                                    sg.Slider((100, 250), 170, orientation='h', key='SLIDER_V', enable_events=True,
                                              size=(11, 11))]], element_justification='c')

    left_img_col = sg.Column([[sg.Image(filename='', key='image_left')]], element_justification='c')

    right_img_xy_col = sg.Column([[sg.Image(filename='', key='image_right')],
                                  [sg.Image(filename='', key='xy_display')]], element_justification='c')

    layout = [[sg.Push(), sg.Frame('Cameras', [[left_camera_col, right_camera_col]]),
               sg.Frame('Color Bounds', [[color_bounds_col1, color_bounds_col2]]), sg.Push()],
              [sg.Push(), sg.Frame('Colors and Borders', [[colors_col, read_col]]), sg.Frame('', [[distance_col]]),
               sg.Push()],
              [sg.Push(), left_img_col, right_img_xy_col, sg.Push()],
              [sg.Push(), sg.Frame('Play', [[play_func]]), sg.Frame('State 1', [[sg.Text('', key='state_1')]]),
               sg.Frame('State 2', [[sg.Text('', key='state_2')]]), sg.Push()]]

    def __init__(self):
        """
        Initializes the gui.
        """
        self.window = None

    def show_gui(self):
        """
        Prepares the qui window to show.
        """
        self.window = sg.Window('FlyBall', Gui.layout, finalize=True, margins=(0, 0), return_keyboard_events=True)

    def update_val(self, left_cam, right_cam, drone_1, drone_2, distance, values):
        """
        Updates values of configurable variables (cameras indices, distance between cameras, drone's homes).
        :param left_cam: the left camera's Camera.
        :param right_cam: the right camera's Camera.
        :param drone_1: the first Drone.
        :param drone_2: the second Drone.
        :param distance: the current distance between the cameras.
        :param values: the values to set.
        :return: the distance between the cameras.
        """
        if values['left_index'] != '':
            left_cam.index = int(values['left_index'])
            left_cam.vid = None

        if values['right_index'] != '':
            right_cam.index = int(values['right_index'])
            right_cam.vid = None

        if values['set_home_1'] != '' or values['set_home_2'] != '':
            self.update_homes(drone_1, drone_2, values)

        if values['distance'] != '':
            distance = int(values['distance'])

        return distance

    def update_homes(self, drone_1, drone_2, values):
        """
        Updates the drones homes.
        :param drone_1: the first drone.
        :param drone_2: the second drone.
        :param values: the values of configurable variables.
        """
        if values['set_home_1'] != '':
            x, y = values['set_home_1'].split(',')
            drone_1.home = (float(x), float(y))
        if values['set_home_2'] != '':
            x, y = values['set_home_2'].split(',')
            drone_2.home = (float(x), float(y))
        self.show_homes_gui(drone_1.home, drone_2.home)

    def show_homes_gui(self, home_1, home_2):
        """
        Shows the location of the homes of the drones.
        :param home_1: the home of the first drone.
        :param home_2: the home of the second drone.
        """
        self.window['home_1'].update("Home 1 = ({:.3f},{:.3f})".format(*home_1))
        self.window['home_2'].update("Home 2 = ({:.3f},{:.3f})".format(*home_2))

    def display_frames_gui(self, balloon, drones, left, right, borders):
        """
        Displays the frames in the gui
        :param balloon: the balloon's RecognizableObject.
        :param drones: a list of the Drones.
        :param left: the left Camera.
        :param right: the right Camera.
        :param borders: the Borders.
        """
        left_img, right_img, xy_display = display_frames(balloon, drones, left, right, borders)
        left_img = self.downscale_image(left_img, 67)
        right_img = self.downscale_image(right_img, 33)
        xy_display = self.downscale_image(xy_display, 33)
        imgbytes_left = cv2.imencode('.png', left_img)[1].tobytes()
        self.window['image_left'].update(data=imgbytes_left)
        imgbytes_right = cv2.imencode('.png', right_img)[1].tobytes()
        self.window['image_right'].update(data=imgbytes_right)
        imgbytes_xy = cv2.imencode('.png', xy_display)[1].tobytes()
        self.window['xy_display'].update(data=imgbytes_xy)

    def update_hsv(self, recognizable_objects, event, values):
        """
        Updates the HSV values for the objects.
        :param recognizable_objects: a list of the RecognizableObjects.
        :param event: the event occurred.
        :param values: the values to set,
        """
        if event == 'SLIDER_H':
            new_h = int(values['SLIDER_H'])
            self.update_h(recognizable_objects, new_h)
        elif event == 'SLIDER_S':
            new_s = int(values['SLIDER_S'])
            self.update_s(recognizable_objects, new_s)
        elif event == 'SLIDER_V':
            new_v = int(values['SLIDER_V'])
            self.update_v(recognizable_objects, new_v)
        elif event == 'SLIDER_T':
            new_tres = int(values['SLIDER_T'])
            self.update_tres(recognizable_objects, new_tres)

    def update_h(self, recognizable_objects, new_h):
        """
        Updates the H value (from HSV) for the objects.
        :param recognizable_objects: a list of the RecognizableObjects.
        :param new_h: the new H value.
        """
        for recognizable_object in recognizable_objects:
            recognizable_object.frame_left.h_range = new_h
            recognizable_object.frame_right.h_range = new_h

    def update_s(self, recognizable_objects, new_s):
        """
        Updates the S value (from HSV) for the objects.
        :param recognizable_objects: a list of the RecognizableObjects.
        :param new_s: the new S value.
        """
        for recognizable_object in recognizable_objects:
            recognizable_object.frame_left.s_range = new_s
            recognizable_object.frame_right.s_range = new_s

    def update_v(self, recognizable_objects, new_v):
        """
        Updates the V value (from HSV) for the objects.
        :param recognizable_objects: a list of the RecognizableObjects.
        :param new_v: the new V value.
        """
        for recognizable_object in recognizable_objects:
            recognizable_object.frame_left.v_range = new_v
            recognizable_object.frame_right.v_range = new_v

    def update_tres(self, recognizable_objects, new_tres):
        """
        Updates the color recognition threshold.
        :param recognizable_objects: a list of the RecognizableObjects.
        :param new_tres: the new threshold value.
        """
        for recognizable_object in recognizable_objects:
            image_left = recognizable_object.frame_left.frame.image
            recognizable_object.frame_left.frame.threshold_size = image_left.shape[1] // new_tres
            image_right = recognizable_object.frame_right.frame.image
            recognizable_object.frame_right.frame.threshold_size = image_right.shape[1] // new_tres

    def update_states(self, state_1, state_2):
        """
        Updates the states of the drones on the view.
        :param state_1: the state of the first drone.
        :param state_2: the state of the second drone.
        """
        self.window['state_1'].update(state_1)
        self.window['state_2'].update(state_2)

    def downscale_image(self, image, scale_percent):
        """
        Downscales an image before shown in the gui window.
        :param image: the image to be shown.
        :param scale_percent: the percentage of down scaling.
        :return: the resized window.
        """
        width = int(image.shape[1] * scale_percent / 100)
        height = int(image.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

        return resized

    def update_borders_button(self, index):
        """
        Updates the borders button according to the index of corners set.
        :param index: the new index of the corners.
        """
        self.window['Set Borders'].update('Set Borders (' + str(index) + ')')
