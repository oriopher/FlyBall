import PySimpleGUI as sg

class Gui:
    colors_column = [[sg.Button('Drone color (L)'), sg.Button('Drone color (R)')],
                    [sg.Button('Balloon color (L)'), sg.Button('Balloon color (R)')]]

    read_column =  [[sg.Button('Set Borders'), sg.Button('Read Colors'), sg.Button('Read Borders')],
                    [sg.Text('Distance:'), sg.InputText(size=(4, 1)), sg.Button('Ok', visible=False, bind_return_key=True), sg.Text('Middle = ', key='-MIDDLE-')]]            

    Tello_func = [[sg.Button('Connect'), sg.Button('Take Off'), sg.Button('Flip'), sg.Button('Land')]]
    play_func = [[sg.Button('Start Track'), sg.Button('Stop Track'), sg.Button('Hit'), sg.Button('Quit')]]

    left_camera_col = [[sg.Text('Left Camera:')],
                        [sg.Listbox(['Efrat Phone', 'Efrat Web', 'Ori Phone', 'Nir Phone', 'Maya Phone'], size=(15,6), select_mode='extended')]]

    right_camera_col = [[sg.Text('Right Camera:')],
                        [sg.Listbox(['Efrat Phone', 'Efrat Web', 'Ori Phone', 'Nir Phone', 'Maya Phone'], size=(15,6))]]

    layout = [  [sg.Text('Settings Functions:', justification='center', size = (57,1), font=('MS Sans Serif', 10, 'bold'), text_color='midnight blue')],
                    [sg.Push(), sg.Column(left_camera_col, element_justification='c'), sg.Push(), sg.Column(right_camera_col, element_justification='c'), sg.Push()],
                    [sg.Column(colors_column, element_justification='c'), sg.VSeparator(), sg.Column(read_column, element_justification='c')],
                    [sg.Text('Tello Functions:', justification='center', size = (57,1), font=('MS Sans Serif', 10, 'bold'), text_color='midnight blue')],
                    [sg.Push(), sg.Column(Tello_func, element_justification='c'), sg.Push()],
                    [sg.Text('Play Functions:', justification='center', size = (57,1), font=('MS Sans Serif', 10, 'bold'), text_color='midnight blue')],
                    [sg.Push(), sg.Column(play_func, element_justification='c'), sg.Push()]]


    def __init__(self):
        #sg.theme('LightGreen')   # Add a touch of color
        self.window = sg.Window('FlyBall', Gui.layout)  # Create the Window
    