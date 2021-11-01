# ArgosCam
This project is the result of my bachelor degree final projects, for my studies in mechatronical engineering, a copy of the complete documentation of the project can be found in the documentation folder. <!-- link a carpeta -->


## Introduction

The idea of this project is the development and prototyping of a system to increase the visibility of drivers at street intersections to reduce the number of accidents that occur in these areas.

Reduced visibility increases the risk of collisions with vehicles approaching the perpendicular street to which the driver is going. Especially if the driver has to invade the road, he wants to incorporate, with his car to have visibility. This problem occurs mainly in rural areas where the streets are narrower and at the exits of the parking lots where many times the car has to cross the sidewalk to enter the street, creating a risk of running over a pedestrian.

The project develops a system to be called ARGOS CAM, referring to Greek mythology in Argos Panoptes.

ARGOS Cam consists of two main parts: one that is responsible for the collection of vehicle data (i.e. speed) and a second part that is responsible for capturing images. This second module - focused on image processing - is responsible for offering help to the driver, through image recognition, and is modulated by the first, which focuses more on aspects of vehicle electronics. The figure 1, reperesents the pipeline of the project.

<p align="center">
  <img alt="pipeline" width="300" height="200" src="https://user-images.githubusercontent.com/61985816/138732579-608c7c00-99d3-459c-a0e8-55cab31e78ad.png">
</p>

Figure 1: pipeline of the project

In Figure 2 you can see an image that represents the advantage of the system for the driver, if two cameras are placed at the front of the car.

<p align="center">
  <img alt="distance_advantage" width="300" height="200" src="https://user-images.githubusercontent.com/61985816/138733064-4024bb10-b107-4e8c-9117-04177adb0b66.jpg">
</p>

Figure 2: Distance between the driver and the front of the car

This method has some advantages in front of traditionals methods, like convex mirrors, because it's directly attached to the car and can be adjusted, also the device notifiy the user of vehicles and persons approching the car.


## Methodology
### Vehicle data collection
In order to collect the data from the car an OBD II reader will be used. This devices connects diretly to the OBD Port of the car. The speed is the only value collected from the car and will be used to turn on and off the screen, depending if the speed is below a threshold by default stablished to 2 km/h. This is done to avoid creating distractions to the driver. 

<p align="center">
  <img alt="USB Obd reader" width="200" height="200" src="https://user-images.githubusercontent.com/61985816/138735380-4f0a0ae7-a241-4c9a-826b-712bd5aaf897.png">
</p>

Figure 4: USB OBD II Reader

The one I have used is this [OBD Reader](https://www.amazon.es/dp/B08DG4CFH8/ref=cm_sw_em_r_mt_dp_6RFKPJA6RXC36EJ8QV0E).


### Image Capture
Any machine vision system has a first phase of image acquisition. For this task I have used two cameras **Raspberry piCameras V2** of 8 Megapixels. This are placed at the front of the car, in figure 5 you can see the point where they are placed.

<p align="center">
  <img alt=camera position" width="200" height="200" src="https://user-images.githubusercontent.com/61985816/138736895-1e6dd7e5-1af0-41ec-a0c9-0c9252dc4029.jpeg">
</p>

Figure 5: Position of Cameras

Due to the intendet purpose of the project is to be used inside cities and towns, the maximum speed, adding some margin to be safe, will be set to 60 Km/h, this is equivalent to travel 17,7 m/s. If this value is converted into frames per second(FPS), and the camera captures at least 18 images(frames), the object would have moved 1 m. At bigger framerates less distance.

<p align="center">
  <img alt=intersections examples" width="300" height="300" src="https://user-images.githubusercontent.com/61985816/138737656-755b64a1-66d8-47fe-8b09-2062977adbac.png">
</p>

Figure 6: Examples of taken images


### Embedded system
The embedded system selected for this project is the [Jetson nano B01](https://www.nvidia.com/es-es/autonomous-machines/embedded-systems/jetson-nano/) from the company **Nvidia**. This board has the advantage of beeing able to acces the **CUDA** library of Nvidia and also has a **GPU**, this makes it perfect for computer vision aplications as it has a better performance than a **Raspberry Pi 4**. 

<p align="center">
  <img alt="jetson nano" width="200" height="200" src="https://user-images.githubusercontent.com/61985816/138739647-c72c4bf9-e7b7-4f79-9b2b-777fbbb59164.png">
</p>

Figure 7: Nvidia Jetson Nano B01

The OS version I used for the project was Jetpack 4.5

The programming language used is python. A detailed explanation of the libraries used can be found in the READ_ME file inside the folder called [Code](https://github.com/ramon-angosto/ArgosCam/tree/main/code).

<p align="center">
  <img alt="Jetson connections" width="300" height="200" src="https://user-images.githubusercontent.com/61985816/138739706-97fdc6f1-32b2-446a-a2d2-1f27819f2863.png">
</p>

Figure 8: Representation of component connections with the Jetson Nano. The materials used for the project are: two Raspberry Pi v2 cameras, used for image capture; a touch screen to view images and interact with the user interface; an OBD reader used for vehicle data collection; a portable battery of the Xiaomi brand, with power capacity of 5v 2.6A;for GPIO connections, four LEDs and a buzzer are used (with the necessary additional components, explained on the READ me file inside the folder called electronics) <!-- link a carpeta -->


### Background subtraction
Images acquired by front-facing cameras are processed through the Background Subtraction algorithm to isolate those objects that are moving in the scene. This is done to avoid detecting parked cars and mislabel static elements.

The algorithm used is **K-NN Background subtraction**, and can be found inside the **OpenCV library**

The selected algorithm is based on the Gaussian Mixture Model (GMM), and el K-nearest neighbor (K-NN). This is described in depth [here](https://www.sciencedirect.com/science/article/abs/pii/S0167865505003521). Generally speaking, a recursive method is presented to update the GMM parameters and update them for each pixel.

Model initialization is performed with the number of frames assigned to the NSamples variable. During this period, you define which objects belong to the background class and which objects of the moving object class by comparing the pixels in these frames. The model is then updated every N of frames, where N is the number of frames that have been defined in the History and parameter.

After the background sustraction is applied, the results goes throug a series of postprocessing steps (Backround sustraction image -> Blur -> Morphological transformations(Opening -> Closing -> Dilate) -> Contour selection).

<p align="center">
  <img alt="Background sustraction 'algorithm' steps" width="300" height="300" src="https://user-images.githubusercontent.com/61985816/138741983-ca453c3d-6c85-4748-b370-5bd44803ff98.png">
</p>

Figure 9: Background sustraction "algorithm" steps

This allow to obtain the desired result. In this case a rectangular shaped masked arround the moving objects.

 Result examples:
 <p align="center">
  <img alt="Background sustraction results" width="300" height="500" src="https://user-images.githubusercontent.com/61985816/139731540-be407ca7-33ee-4cfd-aebd-a8f562b3e667.png">
</p>


### Object detection
After the Background Subtraction, object detection is performed. Thanks to the previous step, static objects, such as parked cars, will not be recognized. This prevents the driver from being misreported, who must be alert to objects that may pose a hazard (for example, an approaching car or a pedestrian crossing the street).

To perform this task, the Mobilenet-SSD v2 network will be used pre-trained with the [COCO(Comon Object in Context)](https://cocodataset.org/#home) data set, being able to recognize up to 90 different objects and the background [35]. This network consists of two main components: MobileNet , a deep neural network with an efficient architecture designed to be used in devices with less computing capacity such as phones and embedded systems, and an SSD (Single Shot Multibox Detector) network, which allows the simultaneous detectionof multiple objects in an image.

The objects that are recognized, i.e., the permitted classes, are: vehicles (bicycles, buses, cars, motorcycles) and people. If an object is not in the classes mentioned above, it would not be detected.

When any object of the set objects is recognized, for the first time since the car stops at an intersection, a noise will sound to warn the driver. This will not sound again until some class is no longer detected while the vehicle is still stopped or once the car moves again and stops at a new intersection.

In addition to the audible warning, the device has light warnings to identify if what has been detected is a pedestrian or a vehicle and which side it is on. This will remain active as long as one of the allowed classes is detected. In the next section you can see in more detail how you will see the output that the driver will receive.

<p align="center">
  <img alt="Detection representation" width="200" height="200" src="https://user-images.githubusercontent.com/61985816/138743085-07106187-f787-4f60-9413-446576661a0c.png">
</p>

Figure 10: Detection representation.


## User interface and alerts
### GUI
In order to create an interface to allow the driver to interact with the device using the screen a UI has been created using PysimpleGUI. The user will be able to interact with the GUI while the car is moving at a speed of less than 2 km/h or is stopped, and will be able to decide if both cameras are used or only one. In addition to this, a button has been incorporated that allows you to disable the recognition of objects in case the driver only wants to have the cameras active and not the warnings. Below are the different screens that the user will see:

<p align="center">
  <img alt="GUI screens" width="300" height="200" src="https://user-images.githubusercontent.com/61985816/138745036-e42c8fed-9740-455e-8cb0-df5dadb96ba1.png">
</p>

Figure 11: GUI screens. A) Charging screen B) Screen while the vehicle is in motion C) Screen with a single camera D) Screen with both cameras


### Luminous and sound warning
The luminous and sound warnings will activated when any of the relevant objects are recognized. There are four light signals, 2 on each side. In this way, the driver can quickly know on which side, the object has been detected. Light signals are divided into two categories: people and vehicles (cars, motorcycles, bicycles and trucks).


### 3D Design
To encapsulate the project, a housing has been designed for the screen, battery, recessed system, and electronic board, and a different one for each camera. This task was carried out using the CAD Fusion 360 program. In the folder called 3D, the following files are included: instruccions to build it, shematics, and the stl. files.

<p align="center">
  <img alt="ARGOS CAM" width="300" height="300" src="https://user-images.githubusercontent.com/61985816/138745712-3d5959ed-3e7d-40c7-977d-75365cefbb19.jpeg">
</p>

Figure 12:  ARGOS CAM system mounted on a car

## A video of the project can be found [here](https://youtu.be/hri5eU1GCwE)

