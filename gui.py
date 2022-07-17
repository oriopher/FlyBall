import PySimpleGUI as sg

class Gui:
    layout = [  [sg.Text('Setting Functions:')],
                [sg.Button('Drone color (L)'), sg.Button('Drone color (R)'), sg.Button('Balloon color (L)'), sg.Button('Balloon color (R)'), sg.Button('Set Borders')],
                [sg.Button('Read Colors'), sg.Button('Read Borders')],
                [sg.Text('Tello Functions:')],
                [sg.Button('Connect'), sg.Button('Take Off'), sg.Button('Flip'), sg.Button('Land')],
                [sg.Text('Play Functions:')],
                [sg.Button('Start Track'), sg.Button('Stop Track'), sg.Button('Hit'), sg.Button('Quit')]]


    def __init__(self):
        sg.theme('LightGreen')   # Add a touch of color
        self.window = sg.Window('FlyBall', Gui.layout)  # Create the Window
    