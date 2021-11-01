# Part of this code is based on Jetson hacks code to control the cameras using threads
# MIT License
# Copyright (c) 2019,2020 JetsonHacks
# See license
# A very simple code snippet
# Using two  CSI cameras (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit (Rev B01) using OpenCV
# Drivers for the camera and OpenCV are included in the base image in JetPack 4.3+

# This script will open a window and place the camera stream from each camera in a window
# arranged horizontally.
# The camera streams are each read in their own thread, as when done sequentially there
# is a noticeable lag
# For better performance, the next step would be to experiment with having the window display
# in a separate thread


# Import needed libraries
import threading
import time
import RPi.GPIO as GPIO
import cv2
import cv2 as cv
import jetson.inference as inference
import jetson.utils as utils
import numpy as np


# class creation to control cameras
class vStream:
    # class initialization with parameters. (Network used, pins used for this camera, frame size)
    def __init__(self, network, width, height, board, pin_led_person, pin_led_car, pin_buzzer):

        ##############################
        # GPIO#
        # parameters used to control GPIO
        self.board = board
        self.pin_led_person = pin_led_person
        self.pin_led_car = pin_led_car
        self.pin_buzzer = pin_buzzer

        # initial state of GPIO pins
        self.board.setup(self.pin_led_car, self.board.OUT, initial=self.board.LOW)
        self.board.setup(self.pin_led_person, self.board.OUT, initial=self.board.LOW)
        self.board.setup(self.pin_buzzer, self.board.OUT, initial=self.board.LOW)
        self.frames_safety_margin = 5  # Safety margin to avoid constant change of state of LEDs or buzzer
        self.buzzer_sound_time = 2 # Number of frames that controls the time the buzzer is producing a noise

        ##############################
        # Camera parameters #
        # frame size definition
        self.width = width
        self.height = height
        self.capture = None
        # Initialize capture parameter

        # Last frame grabbed variable, to store it
        self.frame = None
        self.frame2 = None
        self.grabbed = False
        self.running = True

        ##############################
        # Object recognition#
        self.net = network

        # CHANGE TO REDUCE CONFIDENCE OF OBJECT RECOGNIZED AS AN VALID OBJECT
        self.thr = 0.5  # threshold, 0.5 default value 

        # Names of recognized objects and groups
        self.classes = ['background', 'truck', 'bicycle', 'bus', 'car', 'motorbike', 'person']
        self.classNames = ['person']
        self.classVehicle = ['bicycle', 'truck', 'bus', 'car', 'motorbike']

        # Parameters to control state of GPIO using object recognition
        self.led_person_On = False
        self.led_vehicle_On = False
        self.detected_person = False
        self.detected_vehicle = False

        # Used in combination with variable frames_safety_margin, to control GPIO state
        self.counter_detected_person = 0  # counts the time a person is detected
        self.counter_not_detected_person = 0
        self.counter_detected_vehicle = 0  # counts the time a vehicle is detected
        self.counter_not_detected_vehicle = 0

        ##############################
        # Background subtraction #
        self.box_increase = 15  # increase of bounding box size to ensure moving object is properly recognized later
        self.color = (255, 255, 255)

        # background subtraction algorithm definition and parameters adjustments
        self.fgbg = cv2.createBackgroundSubtractorKNN()
        self.fgbg.setHistory(100)
        self.fgbg.setNSamples(10)
        self.fgbg.setkNNSamples(3)

        # shape and size of kernels used in the project
        self.kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5))
        self.kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        ##############################
        # Thread #
        # parameters to control the thread of the camera
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    # Camera initialization
    def open(self, gstreamer_pipeline_string):
        try:
            # The video controller is GStreamer
            self.capture = cv2.VideoCapture(gstreamer_pipeline_string, cv2.CAP_GSTREAMER)
            self.grabbed, self.frame = self.capture.read()

        except RuntimeError:  # Warns of error found
            self.capture = None
            print("Impossible to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)
            return

    # Function to start thread and call function to start capturing frames
    def start(self):
        if self.running:
            print("Already capturing frames")
            return None

        if self.capture is not None:
            self.running = True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.daemon = True
            self.read_thread.start()
        return self

    # Function used to stop the thread
    def stop(self):
        self.running = False
        self.read_thread.join()

    # Function to capture new frames, this function is called by the thread
    def updateCamera(self):
        while self.running:
            # try grabbing new frames
            try:
                grabbed, frame = self.capture.read()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
                    try:
                        # change the frame capture size to neural network input size
                        self.frame2 = cv2.resize(self.frame, (300, 300))

                    except Exception as err:  # check for errors
                        print("camera capture failed", err)

            except RuntimeError:
                print("Impossible to read new frame")

    # Returns the captured frame, without processing and sets the GPIO to low
    def getFrame(self):
        print("frame")
        self.board.output(self.pin_led_car, self.board.LOW)
        self.board.output(self.pin_led_person, self.board.LOW)
        self.board.output(self.pin_buzzer, self.board.LOW)
        return self.frame2

    # Function used to apply the background subtraction to the captured frame
    def Background_subtraction(self):
        try:
            self.blur = cv2.GaussianBlur(self.frame2, (15, 15), 0)  # Blurs frame
            self.fgmask = self.fgbg.apply(self.blur)  # Apply background subtraction

            # Morphological transformations #
            # clean up of output of background subtraction
            _, self.binary = cv2.threshold(self.fgmask, 50, 255, cv2.THRESH_BINARY)  # Ensures mask is binary
            self.open = cv2.morphologyEx(self.binary, cv2.MORPH_OPEN, self.kernel)  # Reduce image noise
            self.close = cv2.morphologyEx(self.open, cv2.MORPH_CLOSE, self.kernel1)  # Close holes in binary mask
            self.dilate = cv2.morphologyEx(self.close, cv2.MORPH_DILATE, self.kernel1)
            # Expands the mask closing holes and mask more parts of the object

            # Find contours #
            # In case of one object only applied once, if multiple objets are detected it's applied twice
            # This is done to merge different small contours that might belong to one object
            self.contours, self.hierarchy = cv.findContours(self.dilate, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            self.contours_poly = [None] * len(self.contours)
            self.boundRect = [None] * len(self.contours)
            for self.i, self.c in enumerate(self.contours):
                self.contours_poly[self.i] = cv.approxPolyDP(self.c, 3, True)
                self.boundRect[self.i] = cv.boundingRect(self.contours_poly[self.i])

            self.drawing = np.zeros((self.frame2.shape[0], self.frame2.shape[1], 3), dtype=np.uint8)
            # Blank mask to be filled

            for self.i in range(len(self.contours)):
                cv.rectangle(self.drawing, ((int(self.boundRect[self.i][0]) - self.box_increase),
                                            (int(self.boundRect[self.i][1])) - self.box_increase),
                             ((int(self.boundRect[self.i][0] + self.boundRect[self.i][2]) + self.box_increase),
                              (int(self.boundRect[self.i][1] + self.boundRect[self.i][3])) + self.box_increase),
                             self.color, cv2.FILLED)

            self.draw = cv2.cvtColor(self.drawing, cv2.COLOR_BGR2GRAY)  # New mask obtained from find contours

            # Second find contours
            self.contours_, self.hierarchy_ = cv.findContours(self.draw, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            self.contours_poly = [None] * len(self.contours_)
            self.boundRect = [None] * len(self.contours_)
            self.filter = 20  # Contour size filter
            self.pass_filter = []  # list of contours that are bigger than the filter size
            for self.j, self.co in enumerate(self.contours_):
                self.area = cv2.contourArea(self.co)
                if self.area > self.filter:
                    self.pass_filter.append(self.j)
                    self.contours_poly[self.j] = cv.approxPolyDP(self.co, 3, True)
                    self.boundRect[self.j] = cv.boundingRect(self.contours_poly[self.j])

            self.drawing_ = np.zeros((self.frame2.shape[0], self.frame2.shape[1], 3), dtype=np.uint8)

            # check if more than one contour is present
            if len(self.contours_) > 0:
                for self.h in self.pass_filter:
                    cv.drawContours(self.drawing_, self.contours_poly, self.h, self.color)
                    cv.rectangle(self.drawing_,
                                 ((int(self.boundRect[self.h][0]) - self.box_increase),
                                  (int(self.boundRect[self.h][1])) - self.box_increase), \
                                 ((int(self.boundRect[self.h][0] + self.boundRect[self.h][2]) + self.box_increase),
                                  (int(self.boundRect[self.h][1] + self.boundRect[self.h][3])) + self.box_increase),
                                 self.color, cv2.FILLED)

                self.res_box = cv2.bitwise_and(self.frame2, self.frame2,
                                               mask=cv2.cvtColor(self.drawing_, cv2.COLOR_BGR2GRAY))

            else:
                self.res_box = cv2.bitwise_and(self.frame2, self.frame2,
                                               mask=cv2.cvtColor(self.drawing, cv2.COLOR_BGR2GRAY))

            return self.res_box

        except Exception as error_bg:  # check for errors
            print("error background subtraction ", error_bg)

    # function used to apply object recognition with background subtraction
    def recognition_with_bg(self):
        try:
            # calls background subtraction function
            self.res_box = self.Background_subtraction()
            self.backgroud_resized = cv2.resize(self.res_box, (300, 300))  # resize image
            self.copia_frame = self.frame2.copy()
            self.backgroud_resized = cv2.cvtColor(self.backgroud_resized, cv2.COLOR_BGR2RGBA)  # adds alpha channel

            # tranformation of image to CUDA format
            self.backgroud_resized = utils.cudaFromNumpy(self.backgroud_resized)
            self.cols = self.backgroud_resized.shape[1]
            self.rows = self.backgroud_resized.shape[0]

            # Object recognition algorithm, optimized with cuda
            self.detections = self.net.Detect(self.backgroud_resized)

            # reset values to non detected
            self.detected_person = False
            self.detected_vehicle = False
            for self.detect in self.detections:  # loop that goes over all the detections
                # parameters of each detection
                self.ID = self.detect.ClassID
                self.item = self.net.GetClassDesc(self.ID)
                self.confidence = self.detect.Confidence

                if self.confidence > self.thr and (self.item in self.classes):  # filter by class and threshold
                    print(self.item)

                    # GPIO Control
                    # If a LED is not already active and the counter is less than the safety margin,
                    # the buzzer is going to emit a sound and the light is going to be turned on
                    if self.item in self.classVehicle:
                        self.detected_vehicle = True  # Set as detected important for the counter
                        if (not self.led_vehicle_On) and self.counter_not_detected_vehicle > self.frames_safety_margin:
                            # If LED not ON
                            print("buzzer sounds")
                            self.board.output(self.pin_buzzer, self.board.HIGH)
                            self.led_vehicle_On = True
                            self.counter_not_detected_vehicle = 0
                            self.board.output(self.pin_led_car, self.board.HIGH)
                            print("vehicle LED ON")

                    if self.item in self.classNames:
                        self.detected_person = True
                        if (not self.led_person_On) and self.counter_not_detected_person > self.frames_safety_margin:
                            # If LED not ON
                            self.led_person_On = True
                            self.counter_not_detected_person = 0
                            self.board.output(self.pin_led_person, self.board.HIGH)
                            print("person LED ON")

                        # self.board.output(self.pin_buzzer, self.board.LOW)

            # counter increase and control of time the buzzer is sounding
            if self.detected_person:  # Detected number increase
                self.counter_detected_person += 1
                if self.counter_detected_person > self.buzzer_sound_time:
                    self.board.output(self.pin_buzzer, self.board.LOW)
                print("detected", self.counter_detected_person)

            else:  # Not detected number increase
                self.counter_not_detected_person += 1
                print("not  detected", self.counter_not_detected_person)
                if self.counter_not_detected_person > self.buzzer_sound_time:
                    self.board.output(self.pin_buzzer, self.board.LOW)

            if self.detected_vehicle:  # Detected number increase
                self.counter_detected_vehicle += 1
                if self.counter_detected_vehicle > self.buzzer_sound_time:
                    self.board.output(self.pin_buzzer, self.board.LOW)
                print("detected", self.counter_detected_vehicle)

            else:  # Not detected number increase
                self.counter_not_detected_vehicle += 1
                print("not  detected", self.counter_not_detected_vehicle)
                if self.counter_not_detected_vehicle > self.buzzer_sound_time:
                    self.board.output(self.pin_buzzer, self.board.LOW)

            # if not detected after the counter is over the safety margin 
            if (not self.detected_vehicle) and self.counter_detected_vehicle > self.frames_safety_margin:
                if self.led_vehicle_On:
                    print("Vehicle LED OFF")
                    self.board.output(self.pin_led_car, self.board.LOW)
                    self.led_vehicle_On = False
                    self.counter_detected_vehicle = 0

            if (not self.detected_person) and self.counter_detected_person > self.frames_safety_margin:
                if self.led_person_On:
                    print("Person LED OFF")
                    self.board.output(self.pin_led_person, self.board.LOW)
                    self.led_person_On = False
                    self.counter_detected_person = 0

            return self.copia_frame, self.frame2  # returns the bg frame and the original
        except Exception as error_rec:  # Check for errors
            print("Recognition Error", error_rec)

    # Function to do the object recognition applied to the original frame
    # Mostly used for testing, if used on street parked vehicles also will be detected
    def recognition_without_bg(self):
        try:
            # Instead of the bg frame uses the frame captured by the camera
            self.backgroud_resized = cv2.resize(self.frame2, (300, 300))
            self.copia_frame = self.frame2.copy()
            self.backgroud_resized = cv2.cvtColor(self.backgroud_resized, cv2.COLOR_BGR2RGBA)

            # tranformation of image to CUDA format
            self.backgroud_resized = utils.cudaFromNumpy(self.backgroud_resized)
            self.cols = self.backgroud_resized.shape[1]
            self.rows = self.backgroud_resized.shape[0]

            # Object recognition algorithm, optimized with cuda
            self.detections = self.net.Detect(self.backgroud_resized)

            # reset values to non detected
            self.detected_person = False
            self.detected_vehicle = False
            for self.detect in self.detections:  # loop that goes over all the detections
                # parameters of each detection
                self.ID = self.detect.ClassID
                self.item = self.net.GetClassDesc(self.ID)
                self.confidence = self.detect.Confidence

                if self.confidence > self.thr and (self.item in self.classes):  # filter by class and threshold
                    print(self.item)

                    # GPIO Control
                    # If a LED is not already active and the counter is less than the safety margin,
                    # the buzzer is going to emit a sound and the light is going to be turned on
                    if self.item in self.classVehicle:
                        self.detected_vehicle = True  # Set as detected important for the counter
                        if (not self.led_vehicle_On) and self.counter_not_detected_vehicle > self.frames_safety_margin:
                            # If LED not ON
                            print("buzzer sounds")
                            self.board.output(self.pin_buzzer, self.board.HIGH)
                            self.led_vehicle_On = True
                            self.counter_not_detected_vehicle = 0
                            self.board.output(self.pin_led_car, self.board.HIGH)
                            print("vehicle LED ON")

                    if self.item in self.classNames:
                        self.detected_person = True
                        if (not self.led_person_On) and self.counter_not_detected_person > self.frames_safety_margin:
                            # If LED not ON
                            self.led_person_On = True
                            self.counter_not_detected_person = 0
                            self.board.output(self.pin_led_person, self.board.HIGH)
                            print("person LED ON")

                        # self.board.output(self.pin_buzzer, self.board.LOW)

            # counter increase and control of time the buzzer is sounding
            if self.detected_person:  # Detected number increase
                self.counter_detected_person += 1
                if self.counter_detected_person > self.buzzer_sound_time:
                    self.board.output(self.pin_buzzer, self.board.LOW)
                print("detected", self.counter_detected_person)

            else:  # Not detected number increase
                self.counter_not_detected_person += 1
                print("not  detected", self.counter_not_detected_person)
                if self.counter_not_detected_person > self.buzzer_sound_time:
                    self.board.output(self.pin_buzzer, self.board.LOW)

            if self.detected_vehicle:  # Detected number increase
                self.counter_detected_vehicle += 1
                if self.counter_detected_vehicle > self.buzzer_sound_time:
                    self.board.output(self.pin_buzzer, self.board.LOW)
                print("detected", self.counter_detected_vehicle)

            else:  # Not detected number increase
                self.counter_not_detected_vehicle += 1
                print("not  detected", self.counter_not_detected_vehicle)
                if self.counter_not_detected_vehicle > self.buzzer_sound_time:
                    self.board.output(self.pin_buzzer, self.board.LOW)

            # if not detected after the counter is over the safety margin
            if (not self.detected_vehicle) and self.counter_detected_vehicle > self.frames_safety_margin:
                if self.led_vehicle_On:
                    print("Vehicle LED OFF")
                    self.board.output(self.pin_led_car, self.board.LOW)
                    self.led_vehicle_On = False
                    self.counter_detected_vehicle = 0

            if (not self.detected_person) and self.counter_detected_person > self.frames_safety_margin:
                if self.led_person_On:
                    print("Person LED OFF")
                    self.board.output(self.pin_led_person, self.board.LOW)
                    self.led_person_On = False
                    self.counter_detected_person = 0

            return self.copia_frame, self.frame2  # returns the original frame and the original
        except Exception as error_rec:  # Check for errors
            print("Recognition Error", error_rec)


# Function to define raspberry pi camera parameters on the jetson nano
def gstreamer_pipeline(
        sensor_id=0,
        sensor_mode=3,
        capture_width=1280,
        capture_height=720,
        display_width=640,
        display_height=480,
        framerate=30,
        flip_method=0,
):
    return (
            "nvarguscamerasrc sensor-id=%d sensor-mode=%d ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor_id,
                sensor_mode,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
    )


# main section to test this code
if __name__ == "__main__":
    flip = 2
    dispW = 640
    dispH = 480

    # camera parameters check
    camSet = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ' \
             '! nvvidconv flip-method=' + str(flip) + ' ! video/x-raw, width=' + str(dispW) + ', height=' + str(dispH) \
             + ', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

    # cam2 = vStream(camSet, dispW, dispH)
    font = cv2.FONT_HERSHEY_SIMPLEX
    startTime = time.time()
    dtav = 0
    print("activate gpio")
    # GPIO.cleanup()
    Jetson = GPIO
    Jetson.setmode(GPIO.BCM)
    # led car left 11 --> 17
    # led person left 12 --> 18
    # buzzer 13 --> 27
    # led person right 19 --> 10
    # led car right 26 --> 7

    pins = [18, 17, 7, 27, 10]
    # set pin as an output pin with optional initial state of HIGH
    Jetson.setup(pins, GPIO.OUT, initial=GPIO.LOW)
    time.sleep(1)

    #Network initialization
    network_ = inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

    # create two VStreamer objects, one for each camera
    cam1 = vStream(network_, dispW, dispH, Jetson, 17, 18, 27)
    cam1.open(gstreamer_pipeline(flip_method=2))
    cam1.start()
    cam2 = vStream(network_, dispW, dispH, Jetson, 10, 7, 27)
    cam2.open(gstreamer_pipeline(sensor_id=1, flip_method=2))
    cam2.start()

    while True:
        try:
            # apply object recognition to both cameras
            start_time1 = time.time()
            fr, myFrame1 = cam1.recognition_with_bg()
            fps1 = round(1.0 / (time.time() - start_time1), 2)
            fr, myFrame2 = cam2.recognition_with_bg()

            cv2.rectangle(myFrame1, (0, 0), (140, 40), (0, 0, 255), -1)
            cv2.putText(myFrame1, str(round(fps1, 1)) + ' fps', (0, 25), font, .75, (255, 255, 255), 2)
            cv2.imshow('ComboCam', myFrame1)
            cv2.imshow('ComboCam1', myFrame2)
            cv2.moveWindow('ComboCam', 0, 0)

        except Exception as error:  # If no frame available
            print(error)
            print('frame not available')

        if cv2.waitKey(1) == ord('q'):  # if q press in keyboard program ends
            cam1.stop()
            cam2.stop()
            cam1.capture.release()
            cam2.capture.release()
            cv2.destroyAllWindows()
            GPIO.cleanup()
            exit(1)
            break
