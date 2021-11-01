"""The purpose of this code is to allow the testing of the project without the need of using a car, there is an slider
to allow the simulation of the speed
"""

import PySimpleGUI as sg
from synchronized_cameras_recognition_modes import vStream, gstreamer_pipeline
from video_algorithms import vStream as videoStream
import cv2
import time
import Jetson.GPIO as GPIO
import jetson.inference as inference
from imutils.video import FileVideoStream

speed_threshold = 2  # CHANGE TO DEFINE MINIMUM SPEED TO ACTIVATE CAMERAS


def make_win1():
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
        [sg.Text('Speed over {} km/h'.format(speed_threshold), size=(40, 1), justification='center', font='Helvetica 20')]]

    # layout buttons
    button_size = 15
    layout5 = [
        [sg.Button('Left', size=(button_size, 1), pad=(2, 2), font='Helvetica 14', visible=True, disabled=True),
         sg.Button('Both', size=(button_size, 1), pad=(2, 2), font='Helvetica 14', visible=True, disabled=True),
         sg.Button('Right', size=(button_size, 1), pad=(2, 2), font='Helvetica 14', visible=True, disabled=True),
         sg.Button(button_text='Activate AI Mode 2', size=(button_size, 1), pad=(2, 2), font='Helvetica 14',
                   visible=True,
                   disabled=True, key='-IA-'),
         sg.Button(button_text='Video', size=(button_size, 1), pad=(2, 2), font='Helvetica 14', visible=True,
                   disabled=True, key='-Mode-'),
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
              [sg.Text('valor slider', key='text', pad=(200, 0), font="Helvetica 15")],
              [sg.Slider(range=(0, 10), pad=(200, 0), orientation='h', key='slider', change_submits=True, size=(34, 20),
                         default_value=1)],
              [sg.Column(layout5, justification="r", element_justification='center', visible=True, key='-COL6-')]]
    
    # Slider used to emulate speed
    # define the window layout
    return sg.Window('Argos Cam', layout, resizable=False, size=(1024, 550), location=(0, 0)).Finalize()


def main():
    window = make_win1()  # creates the GUI

    print("Camera initialization")
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
    camara1 = vStream(net, dispW, dispH, Jetson, 4, 18, 27)
    camara1.open(gstreamer_pipeline(flip_method=2))
    camara1.start()
    camara2 = vStream(net, dispW, dispH, Jetson, 10, 7, 27)
    camara2.open(gstreamer_pipeline(sensor_id=1, flip_method=2))
    camara2.start()

    # Test video settings
    video = videoStream(net, dispW, dispH, Jetson, 10, 7, 27)
    path = "test/test_video_2.mp4"  # path to test video
    fvs = FileVideoStream(path).start()
    f = fvs.read()
    start_video = cv2.imencode('.png', cv2.resize(f, (wide_1_, height_)))[1].tobytes()

    layout_visible = 1  # Layout actualment visisble
    offset = 0
    x = 0

    start_time = time.time()
    time_ = 0

    # progress bar emulation, can be skipped clicking exit one time
    while True:  # Event Loop
        event1, values = window.read(timeout=10)
        if event1 in (None, 'Exit', 'Cancel') or time_ == 30:
            break
        # update the animation in the window
        time_ = int(time.time() - start_time)
        window['progbar'].update_bar(time_ + 1)

    # modifies visible layout
    window[f'-COL1-'].update(visible=False)
    layout_visible = 3
    window[f'-COL{layout_visible}-'].update(visible=True)

    # Enables the buttons to control active camera
    window['Left'].update(disabled=False)
    window['Both'].update(disabled=False)
    window['Right'].update(disabled=False)
    window['-IA-'].update(disabled=False)
    window['-Mode-'].update(disabled=False)

    # Camera size
    dim_1 = (wide_1_, height_)
    dim_2 = (wide_2_, height_)
    old_slider = 10
    layout_in_movement = True
    Mode = True
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

        elif event == "-Mode-":  # If clicked change between camera and video mode for the demo
            if Mode: 
                window['-Mode-'].update('Camera')
                window[f'-COL{layout_visible}-'].update(visible=False)
                print("changing to video")
                layout_visible = 4
                window[f'-COL{layout_visible}-'].update(visible=True)
                video.para_gpio()
                Mode = False

            else: 
                window['-Mode-'].update('Vídeo')
                print("camera")
                Mode = True

        sz_slider = int(values['slider'])  # reads slider value
        if sz_slider != old_slider:
            window.FindElement('text').Update(sz_slider)  # updates slider text value
            
            if sz_slider > speed_threshold:  # if it's over the threshold no cameras are shown, movement layout visible
                window[f'-COL{layout_visible}-'].update(visible=False)
                window['-COL5-'].update(visible=True)
                layout_in_movement = True

            else:  # cameras start working
                window['-COL5-'].update(visible=False)
                window[f'-COL{layout_visible}-'].update(visible=True)
                layout_in_movement = False

        if not layout_in_movement:  # if car moving at speed lower than speed threshold
            if Mode:
                
                # according to visible layout and AI mode the proper camera algorithms are used
                if layout_visible == 2:
                    if intelligent == 2:
                        recognition, frame = camara1.recognition_with_bg()

                    elif intelligent == 1:
                        recognition, frame = camara1.recognition_without_bg()
                    else:
                        frame = camara1.getFrame()
                    # codify image to be used in PysimpleGUI
                    imgbytes = cv2.imencode('.png', cv2.resize(frame, dim_1))[1].tobytes()
                    window['image1'].update(data=imgbytes)

                elif layout_visible == 4:
                    if intelligent == 2:
                        recognition1, frame1 = camara2.recognition_with_bg()

                    elif intelligent == 1:
                        recognition1, frame1 = camara2.recognition_without_bg()

                    else:
                        frame1 = camara2.getFrame()
                    imgbytes1 = cv2.imencode('.png', cv2.resize(frame1, dim_1))[1].tobytes()  # ditto
                    window['image4'].update(data=imgbytes1)

                else:
                    if intelligent == 2:
                        recognition, frame = camara1.recognition_with_bg()
                        recognition1, frame1 = camara2.recognition_with_bg()

                    elif intelligent == 1:
                        recognition, frame = camara1.recognition_without_bg()
                        recognition1, frame1 = camara2.recognition_without_bg()

                    else:
                        frame = camara1.getFrame()
                        frame1 = camara2.getFrame()

                    imgbytes = cv2.imencode('.png', cv2.resize(frame, dim_2))[1].tobytes()
                    imgbytes1 = cv2.imencode('.png', cv2.resize(frame1, dim_2))[1].tobytes()
                    window['image2'].update(data=imgbytes)
                    window['image3'].update(data=imgbytes1)

            else:  # video algorithm
                if fvs.more():
                    try:
                        frame1 = fvs.read()
                        if intelligent == 2:
                            frame1, recognition1 = video.recognition_with_bg(frame1)

                        elif intelligent == 1:
                            recognition1, frame1 = video.recognition_with_bg_demo(frame1)

                        else:
                            # frame1 = video.getFrame()
                            video.para_gpio()
                            print("frame_normal")
                        imgbytes1 = cv2.imencode('.png', cv2.resize(frame1, dim_1))[1].tobytes()  # ditto
                        window['image4'].update(data=imgbytes1)

                    except Exception as e:  # Catch errors
                        print(e)

                else:
                    fvs.stop()
                    fvs = FileVideoStream(path).start()
                    window['-Mode-'].click()
                    print("Vide has finished")

    # clean up, and stop threads
    window.close()
    video.para_gpio()
    fvs.stop()
    camara1.stop()
    camara1.capture.release()
    camara2.stop()
    camara2.capture.release()
    GPIO.cleanup()


# main function
if __name__ == "__main__":
    main()
