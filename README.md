# ArgosCam
This project is the result of my bachelor degree final projects, for my studies in mechatronical engineering, a copy of the complete documentation of the project can be found in the documentation folder.


## Introduction

The idea of this project is the development and prototyping of a system to increase the visibility of drivers at street intersections to reduce the number of accidents that occur in these areas.

Reduced visibility increases the risk of collisions with vehicles approaching the perpendicular street to which the driver is going. Especially if the driver has to invade the road, he wants to incorporate, with his car to have visibility. This problem occurs mainly in rural areas where the streets are narrower and at the exits of the parking lots where many times the car has to cross the sidewalk to enter the street, creating a risk of running over a pedestrian.

The project develops a system to be called ARGOS CAM, referring to Greek mythology in Argos Panoptes.

ARGOS Cam consists of two main parts: one that is responsible for the collection of vehicle data (i.e. speed) and a second part that is responsible for capturing images. This second module - focused on image processing - is responsible for offering help to the driver, through image recognition, and is modulated by the first, which focuses more on aspects of vehicle electronics. The figure 1, reperesents the pipeline of the project.
<img width="1049" alt="Captura de pantalla 2021-10-25 a las 18 16 24" src="https://user-images.githubusercontent.com/61985816/138732579-608c7c00-99d3-459c-a0e8-55cab31e78ad.png">
Figure 1: pipeline of the project

In Figure 2 you can see an image that represents the advantage of the system for the driver, if two cameras are placed at the front of the car.

![distance_advantage](https://user-images.githubusercontent.com/61985816/138733064-4024bb10-b107-4e8c-9117-04177adb0b66.jpg)

Figure 2: Distance between the driver and the front of the car

This method has some advantages in front of traditionals methods, like convex mirrors, because it's directly attached to the car and can be adjusted, also the device notifiy the user of vehicles and persons approching the car.


## Methodology
### Vehicle data collection
In order to collect the data from the car an OBD II reader will be used. This devices connects diretly to the OBD Port of the car. The speed is the only value collected from the car and will be used to turn on and off the screen, depending if the speed is below a threshold by default stablished to 2 km/h. This is done to avoid creating distractions to the driver. 

<img width="91" alt="image" src="https://user-images.githubusercontent.com/61985816/138735380-4f0a0ae7-a241-4c9a-826b-712bd5aaf897.png">

Figure 4: USB OBD II Reader

The one I have used is this [OBD Reader](https://www.amazon.es/dp/B08DG4CFH8/ref=cm_sw_em_r_mt_dp_6RFKPJA6RXC36EJ8QV0E).


### Image Capture
Any machine vision system has a first phase of image acquisition. For this task I have used two cameras **Raspberry piCameras V2** of 8 Megapixels. This are placed at the front of the car, in figure 5 you can see the point where they are placed.

![image](https://user-images.githubusercontent.com/61985816/138736895-1e6dd7e5-1af0-41ec-a0c9-0c9252dc4029.jpeg)

Figure 5: Position of Cameras

Due to the intendet purpose of the project is to be used inside cities and towns, the maximum speed, adding some margin to be safe, will be set to 60 Km/h, this is equivalent to travel 17,7 m/s. If this value is converted into frames per second(FPS), and the camera captures at least 18 images(frames), the object would have moved 1 m. At bigger framerates less distance.

<img width="284" alt="image" src="https://user-images.githubusercontent.com/61985816/138737656-755b64a1-66d8-47fe-8b09-2062977adbac.png">

Figure 6: Examples of taken images


### Embedded system
The embedded system selected for this project is the [Jetson nano B01](https://www.nvidia.com/es-es/autonomous-machines/embedded-systems/jetson-nano/) from the company **Nvidia**. This board has the advantage of beeing able to acces the **CUDA** library of Nvidia and also has a **GPU**, this makes it perfect for computer vision aplications as it has a better performance than a **Raspberry Pi 4**. 


Figure 7: Nvidia Jetson Nano B01

The OS version I used for the project was Jetpack 4.5

The programming language used is python. A detailed explanation of the libraries used can be found in the READ_ME file inside the folder called Code.



