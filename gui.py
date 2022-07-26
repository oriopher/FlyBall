import PySimpleGUI as sg

class Gui:
    colors_column = sg.Column([[sg.Button('Drone color (L)'), sg.Button('Drone color (R)')],
                        [sg.Button('Balloon color (L)'), sg.Button('Balloon color (R)')]], element_justification='c')

    read_column =  sg.Column([[sg.Button('Save Colors'), sg.Button('Set Borders', key='-SetBorders-')],
                    [sg.Button('Load Colors'), sg.Button('Load Borders')]] , element_justification='c')        

    distance_column = sg.Column([[sg.Text('Distance:', key='-DISTANCE-'), sg.InputText(size=(4, 1)), sg.Button('Ok', visible=False, bind_return_key=True)],
                            [sg.Text('Middle = ', key='-MIDDLE-')]], element_justification='c')

    Tello_func = sg.Column([[sg.Button('Connect'), sg.Button('Take Off'), sg.Button('Flip'), sg.Button('Land')]], element_justification='c')

    play_func_col = sg.Column([[sg.Button('Start Track'), sg.Button('Stop Track'), sg.Button('Hit'), sg.Button('Quit')]], element_justification='c')

    left_camera_col = sg.Column([[sg.Text('Left Camera:'), sg.Checkbox('Flip', key='-FLIP_LEFT-')],
                            [sg.Listbox(['Efrat Phone', 'Efrat Web'], size=(17,2), select_mode='extended', key='-LEFT_CAM-')]], element_justification='c')

    right_camera_col = sg.Column([[sg.Text('Right Camera:'), sg.Checkbox('Flip', key='-FLIP_RIGHT-')],
                            [sg.Listbox(['Efrat Phone', 'Efrat Web'], size=(17,2), select_mode='extended', key='-RIGHT_CAM-')]], element_justification='c')

    color_bounds_col1 = sg.Column([[sg.Text('THRESHOLD', font=("Helvetica", 8)), sg.Slider((0, 15), 8, orientation='h',key='-SLIDER_T-', enable_events=True, size=(11, 11))],
                        [sg.Text('H RANGE     ', font=("Helvetica", 8)), sg.Slider((10, 30), 20, orientation='h',key='-SLIDER_H-', enable_events=True, size=(11, 11))]], element_justification='c')

    color_bounds_col2 = sg.Column([[sg.Text('S RANGE     ', font=("Helvetica", 8)), sg.Slider((20, 40), 30, orientation='h',key='-SLIDER_S-', enable_events=True, size=(11, 11))],                  
                        [sg.Text('H RANGE     ', font=("Helvetica", 8)), sg.Slider((100, 250), 170, orientation='h',key='-SLIDER_V-', enable_events=True, size=(11, 11))]], element_justification='c')    


    layout = [ [sg.Frame('Cameras', [[left_camera_col, right_camera_col]]), sg.Frame('Color Bounds', [[color_bounds_col1, color_bounds_col2]])],
                [sg.Push(), sg.Frame('Colors', [[colors_column, read_column]]), sg.Frame('', [[distance_column]]), sg.Push()],
                [sg.Image(filename='', key='image_left'), sg.Image(filename='', key='image_right')],
                [sg.Push(), sg.Frame('Tello', [[Tello_func]]), sg.Frame('Play', [[play_func_col]]), sg.Frame('State', [[sg.Text('', key='-STATE-')]]), sg.Push()]]


    def __init__(self):
        #sg.theme('LightGreen')   # Add a touch of color
        self.window = sg.Window('FlyBall', Gui.layout, finalize=True)  # Create the Window
    