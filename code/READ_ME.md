# Necessary libraries: #
- [PySimpleGUI](https://pysimplegui.readthedocs.io/en/latest/)
- [Jetson.GPIO](https://github.com/NVIDIA/jetson-gpio)
- threading
- [obd](https://python-obd.readthedocs.io/en/latest/)
- [jetson.inference](https://github.com/dusty-nv/jetson-inference)
- [jetson.utils](https://github.com/dusty-nv/jetson-utils)
- queue

### Follow [this instruccions](https://www.pyimagesearch.com/2020/03/25/how-to-configure-your-nvidia-jetson-nano-for-computer-vision-and-deep-learning/) to intall this libraries ###
- Numpy
- OpenCv with CUDA support and contrib modules 

# Files explanation #
All the codes have comments to make them easier to understand

## synchronized_cameras_recognition_modes and video_algorithms ##
Contain the algorithms to control the cameras using threads. Also the functions to apply the background subtraction and object recognition while managing the GPIO pins.

## GPIO_ ##
This simple code can be used to test if the GPIO pins are set correctly without having to execute the complete code

## Simulation_ARGOS_CAM ##
As the name says is a simulator of how the code will perform when integrated on the car. This code dosen't require to use an OBD reader, the speed is emulated using an slider. The other parts of the code work as if they where integrated in the car.

## ARGOS_CAM ##
This code has to be executed when the board is connected to the obd reader, if not it's not going to work. It reads the values of the obd reader and with the obtained values regualte if the cameras capture images or not. The user can choose which camera is shown or if both are shown. 

## test ## 
 Contains two videos that can be tested in the **Simulation_ARGOS_CAM** algorithm.




