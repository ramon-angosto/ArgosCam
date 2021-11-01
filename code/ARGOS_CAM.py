"""This code is meant to be used inside the car in combination with the obd reader
"""
# Import of necessary libraries
import PySimpleGUI as sg
from synchronized_cameras_recognition_modes import vStream, gstreamer_pipeline
import cv2
import time
from obd.protocols import ECU
from obd import OBDStatus
import obd
from queue import Queue
import threading
import Jetson.GPIO as GPIO
import jetson.inference as inference

# sets obd to detect all codes
obd.commands.TIME_SINCE_DTC_CLEARED.ecu = ECU.ALL
que = Queue()

# definition of global variables
speed = 0
high_speed = True
exit_ = True


# Avoid constant activation and deactivation of camera when moving slowly
speed_threshold = 5  # MODIFY TO REGULATE WHEN CAMERAS ARE ACTIVATED


# Function used to create a connection between the board and the OBD reader
def connect(n):
    n = "good"
    quick = True
    ports = obd.scan_serial()  # scans usb ports and return the available ones
    print(ports)  # shows available ports

    # establish the connection with the obd in asynchronous mode, allowing constant reading of values
    connection = obd.Async(ports[0], baudrate=38400, protocol="3", fast=quick, timeout=30)
    while len(
            connection.supported_commands) < 10:  # tris to connect until 10 valid commands are found
        try:
            connection.stop()
            time.sleep(5)  # wait 5 seconds to allow a proper connection
            connection = obd.Async(ports[0], 38400, "3", fast=quick, timeout=30)

        except Exception as error:  # check if an error happened
            print("error while trying to connect: ", error)

    print("finished")
    if OBDStatus.CAR_CONNECTED:  # check again if connection was successful
        print("connected")
    return connection


# function used to get the new value of speed, also to control if the vehicle is in movement or not
def new_value1(current_speed):
    global high_speed
    global exit_
    global speed
    speed = current_speed.value.magnitude  # assign value to speed
    print(speed)
    if speed < speed_threshold:
        high_speed = False

    else:
        high_speed = True


# Function used to control the loading bar in sync with the connection function
def time_(second, window, thread):
    progress = 0
    for i in range(int(second * 10)):
        time.sleep(1)  # sleep for a while
        progress += 100 / (second * 10)
        window['-SEC-'].click()

        if not (thread.is_alive()):  # check if the other thread is still running or has finished
            window['-THREAD-'].click()
            break
        print("threading1", progress)

    window['-THREAD-'].click()
    print("finished_2")  # confirmation of finished thread


def main():
    sg.theme('Reddit')
    # ----------- Layout creation ----------
    # If you want to change the logo of the project do it here
    logo = "logo.png"
    height = 350
    wide_1 = 650
    wide_2 = 490
    im_ = cv2.imread(logo)
    im_ = cv2.imencode('.png', cv2.resize(im_, (height, height)))[1].tobytes()

    #loading layout
    layout1 = [[sg.Image(data=im_, pad=(200, 0), size=(height, height), background_color='white', key='-IMAGE-')],
               [sg.ProgressBar(30, orientation='h', pad=(200, 0), size=(150, 20), key='progbar')]]

    # layout left camera
    layout2 = [
        [sg.Image(filename=logo, key='image1', pad=(200, 0), size=(wide_1, height))],
        [sg.Text('Left camera', size=(40, 1), justification='center', font='Helvetica 20')]]

    # layout both cameras
    layout3 = [[sg.Image(filename=logo, key='image2', size=(wide_2, height)),
                sg.Image(filename=logo, key='image3', size=(wide_2, height))],
               [sg.Text('Both cameras', size=(40, 1), justification='center', font='Helvetica 20')]]

    # layout right camera
    layout4 = [
        [sg.Image(filename=logo, key='image4', pad=(200, 0), size=(wide_1, height))],
        [sg.Text('Right camera', size=(40, 1), justification='center', font='Helvetica 20')]]

    # Layout when speed over threshold
    layout_in_movement_ = [
        [sg.Image(filename=logo, background_color='white', key='-IMAGE2-', pad=(200, 0), size=(wide_1, height))],
        [sg.Text('Speed over {} km/h'.format(speed_threshold), size=(40, 1), justification='center',
                 font='Helvetica 20')]]

    # layout buttons
    button_size = 15
    layout5 = [
        [sg.Button('Left', size=(button_size, 1), pad=(2, 2), font='Helvetica 14', visible=True, disabled=True),
         sg.Button('Both', size=(button_size, 1), pad=(2, 2), font='Helvetica 14', visible=True, disabled=True),
         sg.Button('Right', size=(button_size, 1), pad=(2, 2), font='Helvetica 14', visible=True, disabled=True),
         sg.Button(button_text='Activate AI Mode 2', size=(button_size, 1), pad=(2, 2), font='Helvetica 14',
                   visible=True,
                   disabled=True, key='-IA-'),
         sg.Button('Exit', size=(button_size, 1), pad=(50, 2), font='Helvetica 14', visible=True)]]

    # main layout
    layout = [[sg.Text('ATTENTION! Watch the surroundings for safety', justification="c", font="Helvetica 30",
                       pad=(230, 0), size=(40, 1), key='text_superior')],
              [sg.Column(layout1, justification="t", element_justification='center', key='-COL1-'),
               sg.Column(layout2, justification="t", element_justification='center', visible=False, key='-COL2-'),
               sg.Column(layout3, justification="t", element_justification='center', visible=False, key='-COL3-'),
               sg.Column(layout4, justification="t", element_justification='center', visible=False, key='-COL4-'),
               sg.Column(layout_in_movement_, justification="t", element_justification='center', visible=False,
                         key='-COL5-')],
              [sg.Button("-SEC-", visible=False), sg.Button('-THREAD-', visible=False)],
              [sg.Column(layout5, justification="r", element_justification='center', visible=True, key='-COL6-')]]

    # defines the window with proper dimensions
    window = sg.Window('Argos Cam', layout, resizable=False, size=(1024, 550)).Finalize()

    timeout = thread = None

    accelerate_progress_bar = False
    timeout = None
    t = 50  # max time of initialization
    c = "a"

    # thread used for the loading screen
    thread = threading.Thread(target=lambda q, arg1: q.put(connect(arg1)), args=(que, c), daemon=True)
    thread1 = threading.Thread(target=time_, args=(t, window, thread), daemon=True)
    thread.start()
    thread1.start()

    # element used to continue updating the progress bar after thread ends
    progress_time = 0

    # load screen
    while True:  # Event Loop
        event, values = window.read(timeout=10)
        # if time ends, exit or cancel buttons clicked loop ends
        if event in (None, 'Exit', 'Cancel') or progress_time > 100:
            break

        if event == '-SEC-':  # internal click to update progress bar
            # when the thread ends the progress bar goes faster
            if accelerate_progress_bar:
                progress_time = progress_time + 1
                window['progbar'].update_bar(time_, 100)
                time.sleep(.1)
                window['-SEC-'].click()
            else:
                progress_time = progress_time + 1
            window['progbar'].update_bar(time_, 100)

        # event called when thread ends, saves the connection element
        if event == '-THREAD-':
            # stops both threads
            thread.join(timeout=0)
            connection = que.get()  # saves connection element
            thread1.join(timeout=0)
            print('Thread finished', thread.is_alive(), thread1.is_alive())
            accelerate_progress_bar = True
            window['-SEC-'].click()

    # Sizes of the images used in the layout
    dispW = 640
    dispH = 480
    height_ = 350
    wide_1_ = 650
    wide_2_ = 490
    print("activate gpio")
    # GPIO.cleanup()
    Jetson = GPIO
    Jetson.setmode(GPIO.BCM)
    pins = [18, 4, 7, 27, 10]  # MODIFY ACCORDING TO YOUR PINS
    Jetson.setup(pins, GPIO.OUT, initial=GPIO.LOW)

    # Neural network can be changed here
    network_name = "ssd-mobilenet-v2"
    net = inference.detectNet(network_name, threshold=0.35)
    camera1 = vStream(net, dispW, dispH, Jetson, 4, 18, 27)
    camera1.open(gstreamer_pipeline(flip_method=2))
    camera1.start()
    camera2 = vStream(net, dispW, dispH, Jetson, 10, 7, 27)
    camera2.open(gstreamer_pipeline(sensor_id=1, flip_method=2))
    camera2.start()

    # start reading values from the car
    connection.watch(obd.commands.SPEED, callback=new_value1)
    connection.start()

    window[f'-COL1-'].update(visible=False)
    layout_visible = 3
    window[f'-COL{layout_visible}-'].update(visible=True)

    # Enables the buttons to control active camera
    window['Left'].update(disabled=False)
    window['Both'].update(disabled=False)
    window['Right'].update(disabled=False)
    window['-IA-'].update(disabled=False)

    # Camera size
    dim_1 = (wide_1_, height_)
    dim_2 = (wide_2_, height_)
    layout_in_movement = True
    intelligent = 1  # 0 deactivate, 1 demo, 2 real

    # main loop
    while True:
        event, values = window.read(timeout=20)

        # if exit button, window closed or keyboard key q pressed program ends
        if event == 'Exit' or event == sg.WIN_CLOSED or cv2.waitKey(1) == ord('q'):  #
            break

        elif event == 'Left':  # if left button pressed, this will be the only camera used and visible
            window[f'-COL{layout_visible}-'].update(visible=False)
            layout_visible = 2
            window[f'-COL{layout_visible}-'].update(visible=True)

        elif event == 'Both':   # Both cameras will be used and shown
            window[f'-COL{layout_visible}-'].update(visible=False)
            layout_visible = 3
            window[f'-COL{layout_visible}-'].update(visible=True)

        elif event == 'Right':  # if left button pressed, this will be the only camera used and visible
            window[f'-COL{layout_visible}-'].update(visible=False)
            layout_visible = 4
            window[f'-COL{layout_visible}-'].update(visible=True)

        elif event == '-IA-':  # By clicking this button the IA method is selected
            if intelligent == 0:  # IA deactivated
                window['-IA-'].update('Activate AI Mode 2')
                intelligent = 1

            elif intelligent == 1:  # IA Method 1
                window['-IA-'].update('Deactivate IA')
                intelligent = 2

            else:  # IA Method 2
                window['-IA-'].update('Activate DEMO Mode')
                intelligent = 0

        if high_speed:
            # modify the layout according to vehicle speed
            # this is done automatically using values acquired using obd reader
            window[f'-COL{layout_visible}-'].update(visible=False)
            window['-COL5-'].update(visible=True)
            layout_in_movement = True

        else:  # cameras start working
            window['-COL5-'].update(visible=False)
            window[f'-COL{layout_visible}-'].update(visible=True)
            layout_in_movement = False

        if not layout_in_movement:  # if car moving at speed lower than speed threshold
            # according to visible layout and AI mode the proper camera algorithms are used
            if layout_visible == 2:
                if intelligent == 2:
                    recognition, frame = camera1.recognition_with_bg()

                elif intelligent == 1:
                    recognition, frame = camera1.recognition_without_bg()
                else:
                    frame = camera1.getFrame()
                    
                #codify image to be used in PysimpleGUI
                imgbytes = cv2.imencode('.png', cv2.resize(frame, dim_1))[1].tobytes()
                window['image1'].update(data=imgbytes)

            elif layout_visible == 4:
                if intelligent == 2:
                    recognition1, frame1 = camera2.recognition_with_bg()

                elif intelligent == 1:
                    recognition1, frame1 = camera2.recognition_without_bg()

                else:
                    frame1 = camera2.getFrame()
                imgbytes1 = cv2.imencode('.png', cv2.resize(frame1, dim_1))[1].tobytes()  # ditto
                window['image4'].update(data=imgbytes1)

            else:
                if intelligent == 2:
                    recognition, frame = camera1.recognition_with_bg()
                    recognition1, frame1 = camera2.recognition_with_bg()

                elif intelligent == 1:
                    recognition, frame = camera1.recognition_without_bg()
                    recognition1, frame1 = camera2.recognition_without_bg()

                else:
                    frame = camera1.getFrame()
                    frame1 = camera2.getFrame()

                imgbytes = cv2.imencode('.png', cv2.resize(frame, dim_2))[1].tobytes()
                imgbytes1 = cv2.imencode('.png', cv2.resize(frame1, dim_2))[1].tobytes()
                window['image2'].update(data=imgbytes)
                window['image3'].update(data=imgbytes1)

    # clean up, and stop threads
    window.close()
    camera1.stop()
    connection.close()
    camera1.capture.release()
    camera2.stop()
    camera2.capture.release()
    GPIO.cleanup()


# main function
if __name__ == '__main__':
    main()
