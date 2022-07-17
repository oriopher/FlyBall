import PySimpleGUI as sg

sg.theme('LightGreen')   # Add a touch of color
# All the stuff inside your window.
layout = [  [sg.Image(filename="logo.png")],
            [sg.Text('Setting Functions:')],
            [sg.Button('Drone color (Left)'), sg.Button('Drone color (Right)'), sg.Button('Balloon color (Left)'), sg.Button('Balloon color (Right)'), sg.Button('Set Borders')],
            [sg.Text('Read colors and borders:')],
            [sg.Button('Read Colors'), sg.Button('Read Borders')],
            [sg.Text('Play Functions:')],
            [sg.Button('Start Track'), sg.Button('Stop Track'), sg.Button('Hit'), sg.Button('Land'), sg.Button('Quit')],
            [sg.Button('Ok'), sg.Button('Cancel')] ]

# Create the Window
window = sg.Window('FlyBall', layout)
# Event Loop to process "events" and get the "values" of the inputs
while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED or event == 'Cancel': # if user closes window or clicks cancel
        break

window.close()