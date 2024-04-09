# XArmCubeTracker

## Overview
The XArmCubeTracker project integrates the XArm robotic arm with real-time cube tracking using a camera. The robotic arm is controlled to follow the movement of a cube detected in the camera's field of view. This project is implemented using ROS and requires the XArm repository. 

## Prerequisites
Before using this package, make sure you have the following prerequisites installed:
- ROS (Robot Operating System)
- XArm repository
- OpenCV
- NumPy

## Installation
1. Clone this repository into your ROS workspace's `src` directory:
   ```bash
   git clone https://github.com/andrea2702/XArmCubeTracker.git
   
2. Install the required dependencies:
    ```bash
   sudo apt-get install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport
    pip install numpy opencv-python
    
## Usage

Follow the instructions in the XArm repository to set up and control the XArm robotic arm.

Run the XarmVision script using rosrun:
   ```bash
   rosrun XArmCubeTracker XarmVision
   ```

Place a cube in front of the camera, and the XArm will track its movement in the x and z axes.

## Demo
[![Demo del Proyecto](http://img.youtube.com/vi/LTLvf79X1W8/0.jpg)](https://www.youtube.com/shorts/LTLvf79X1W8)
[![Demo del Proyecto](http://img.youtube.com/vi/Ks-cSf7Eeq8/0.jpg)](https://www.youtube.com/watch?v=Ks-cSf7Eeq8)
