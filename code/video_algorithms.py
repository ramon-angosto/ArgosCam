# Import needed libraries
import threading
import numpy as np
import cv2
import cv2 as cv
import jetson.utils as utils


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
        self.buzzer_sound_time = 2  # Number of frames that controls the time the buzzer is producing a noise

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

    # Funció per obtenir el frame sense cap tipus de processament, manté parats els GPIO
    def para_gpio(self):
        print("atura GPIO")
        self.placa.output(17, self.placa.LOW)
        self.placa.output(18, self.placa.LOW)
        self.placa.output(self.pin_led_cotxe, self.placa.LOW)
        self.placa.output(self.pin_led_persona, self.placa.LOW)
        self.placa.output(self.pin_buzzer, self.placa.LOW)

    # Funcio per aplicar el background susbtraction
    def Background_substraction(self, imatge):
        try:
            self.blur = cv2.GaussianBlur(imatge, (15, 15), 0)  # Difumina l'imatge
            self.fgmask = self.fgbg.apply(self.blur)  # Calcual la mascara de fons
            _, self.binary = cv2.threshold(self.fgmask, 50, 255,
                                           cv2.THRESH_BINARY)  # Asegura que la mascara sigui binaria
            self.open = cv2.morphologyEx(self.binary, cv2.MORPH_OPEN, self.kernel)  # redueix el soroll de la imatge
            self.close = cv2.morphologyEx(self.open, cv2.MORPH_CLOSE,
                                          self.kernel1)  # fa que la mascara quedi mes neta, tanca forats
            self.dilate = cv2.morphologyEx(self.close, cv2.MORPH_DILATE, self.kernel1)  # expandeix la mascara

            # primera detecció de contorns i unica en cas de que hi hagi un sol contorn
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

            self.drawing_ = np.zeros((imatge.shape[0], imatge.shape[1], 3), dtype=np.uint8)

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

                self.res_box = cv2.bitwise_and(imatge, imatge, mask=cv2.cvtColor(self.drawing_, cv2.COLOR_BGR2GRAY))
            else:
                self.res_box = cv2.bitwise_and(imatge, imatge, mask=cv2.cvtColor(self.drawing, cv2.COLOR_BGR2GRAY))

            return self.res_box, imatge

        except Exception as error_bg:  # check for errors
            print("error background subtraction ", error_bg)

    # function used to apply object recognition with background subtraction
    def recognition_with_bg(self, img):
        try:
            # calls background subtraction function
            self.res_box = self.Background_subtraction(img)
            self.backgroud_resized = cv2.resize(self.res_box, (300, 300))  # resize image
            self.copy_frame = self.res_box.copy()
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

            return self.res_box, self.copy_frame  # returns the bg frame and the original
        except Exception as error_rec:  # Check for errors
            print("Recognition Error", error_rec)

    # Function to do the object recognition applied to the original frame
    # Mostly used for testing, if used on street parked vehicles also will be detected
    def recognition_without_bg(self, img):
        try:
            # Instead of the bg frame uses the frame captured by the camera
            self.res_box = cv2.resize(img, (300, 300))
            self.copy_frame = self.res_box.copy()
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

            return self.copy_frame, self.res_box  # returns the original frame and the original
        except Exception as error_rec:  # Check for errors
            print("Recognition Error", error_rec)
