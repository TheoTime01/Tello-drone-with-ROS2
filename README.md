<img src="/images/Humble.png" align="right" width="300" alt="header pic"/>

# Tello Drone with ROS2

![ROS2](https://img.shields.io/badge/ros2-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)

## Authors

- [Théotime PERRICHET](https://github.com/TheoTime01)
- [Tom RECHE](https://github.com/TomRecheEln)
- [Arnaud SIBENALER](https://github.com/ArnaudS-CPE)

## Table of Contents

- [Tello Drone with ROS2](#tello-drone-with-ros2)
  - [Authors](#authors)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Objectives](#objectives)
    - [Demo Video](#demo-video)
  - [Presentation of the Drone](#presentation-of-the-drone)
    - [Trajectory Control](#trajectory-control)
  - [Mission](#mission)
    - [Scenarios to Implement](#scenarios-to-implement)
    - [Behavioral Modes](#behavioral-modes)
  - [Getting Started](#getting-started)
    - [Prerequisites](#prerequisites)
    - [Installation Steps](#installation-steps)
    - [1. Create the ROS2 Workspace](#1-create-the-ros2-workspace)
    - [2. Clone the Project Repository](#2-clone-the-project-repository)
    - [3. Install ROS Dependencies](#3-install-ros-dependencies)
    - [4. Install Python Dependencies](#4-install-python-dependencies)
    - [5. Build the Project](#5-build-the-project)
    - [6. Launch the Drone Controller](#6-launch-the-drone-controller)
    - [QR Code Triggered Behaviors](#qr-code-triggered-behaviors)
  - [⚠️Safety Precautions](#️safety-precautions)
  - [To Improve](#to-improve)


## Introduction

Welcome to my **Tello Drone with ROS2** project! In this master project, I’ll be exploring the capabilities of [ROS2 Humble](https://docs.ros.org/en/humble/index.html) (Robot Operating System 2) using the DJI [Tello EDU](https://www.ryzerobotics.com/tello-edu) drone.

## Objectives

- **Drone Integration**
- **Implement Control Scenarios**
- **Behavioral Programming**


### Demo Video

[![Watch the video](/images/drone%20miniature.png)](https://youtu.be/YJ8u02rbd7Y)

## Presentation of the Drone

The Tello drone is a **Quadrotor UAV** (Unmanned Aerial Vehicle), controlled by four key commands that influence its orientation and movement in space:

- **Thrust**: This command controls the power delivered to the drone's four motors, adjusting the overall thrust. By increasing the throttle, the drone ascends, while reducing it causes the drone to descend. Throttle control affects the drone's altitude.

- **Yaw**: This control rotates the drone about its vertical axis (Z-axis). By adjusting the relative speeds of the motors (clockwise or counterclockwise), I can rotate the drone on its axis, allowing it to turn on itself.

- **Pitch**: Pitch refers to the rotation around the horizontal axis (X-axis). Tilting the drone forward moves it forward, while tilting it backward moves it in reverse.

- **Roll**: Roll is the rotation around the lateral axis (Y-axis). By tilting the drone to the left or right, it moves in the respective direction.

### Trajectory Control

To control the trajectory of the quadrotor drone, I will need to coordinate these four controls simultaneously:

- **Altitude Control**: By increasing or decreasing the throttle, I can adjust the drone's altitude. A uniform increase in the speed of all four motors raises the drone, while reducing the speed lowers it.

- **Yaw Control**: I’ll use yaw to adjust the drone’s direction. For instance, to rotate the drone to the right, I will increase the speed of the counterclockwise motors and decrease the speed of the clockwise motors.

- **Moving Forwards/Backwards**: Adjusting pitch allows me to move the drone forwards or backwards. Tilting the drone forward (by increasing the rear motors’ speed and reducing the front motors' speed) moves it forward, while doing the reverse will move it backward.

- **Lateral Movement**: Roll controls lateral movement. Tilting the drone to the left (by reducing the left-side motors' speed and increasing the right-side motors’ speed) moves the drone to the left, while the reverse moves it to the right.

<div style="display: flex; justify-content: space-between;">
    <img src="/images/drone.jpg" alt="Image 1" style="width: 48%;">
    <img src="/images/done2.jpg" alt="Image 2" style="width: 48%;">
</div>

## Mission

The objective is to set up and implement various scenarios on the Tello EDU drone, encompassing manual control and automated behaviors.

### Scenarios to Implement

1. **Manual Control with Joystick**
   - Control the drone manually using a joystick connected to my PC.

2. **QR Code Triggered Behaviors**
   - Program the drone to execute specific behaviors when a QR code is detected.

### Behavioral Modes

- **Follower Mode**
  - The drone will follow the detected QR code, adjusting its position in real-time to maintain a relative distance or specific alignment to the code.

- **Monitoring Mode**
  - The drone will turn on itself continuously to simulate a monitoring scenario, capturing its surroundings without moving away from its location.

- **Manual Mode**
  - Full manual control of the drone using a xbox joystick.

- **Travelling Mode**
  - When the drone detects the QR code `start`, it will move sideways until it detects the QR code `finish`.

## Getting Started

### Prerequisites

- **Hardware Requirements**
  - DJI Tello EDU drone
  - PC with Wi-Fi capability
  - XBOX joystick controller

- **Software Requirements**
  - ROS2 Humble
  - Python 3.6 or higher
  - OpenCV library for image processing
  - QR code detection libraries (e.g., `pyzbar`)

### Installation Steps

### 1. Create the ROS2 Workspace

If you don't already have a ROS2 workspace, start by creating one.

```bash
# Create a folder for the workspace
mkdir -p ~/ros2_ws/src

# Go to the directory
cd ~/ros2_ws

# Build the workspace
colcon build --symlink-install

# Ensure the environment is properly set up
source install/setup.bash
```

### 2. Clone the Project Repository

Clone your project repository into the `src` folder of the workspace.

```bash
# Go to the src folder
cd ~/ros2_ws/src

# Clone the repository
git clone https://gitlab.com/cpelyon/rob/5eti-2024-2025/iros/S1_G2_Perrichet_Reche_Sibenaler.git

# Return to the workspace
cd ~/ros2_ws
```

### 3. Install ROS Dependencies

Install the ROS dependencies, including the packages needed to handle the controller and QR codes.

```bash
# Install the 'ros-humble-joy' package for the controller
sudo apt install ros-humble-joy

# Install the 'libzbar-dev' library for reading QR codes
sudo apt install libzbar-dev
```

### 4. Install Python Dependencies

```bash
# Install Python dependencies
pip install -r requirements.txt
```

### 5. Build the Project

Compile your project with `colcon`.

```bash
# Build the project
colcon build

# Source the workspace
source install/setup.bash
```

### 6. Launch the Drone Controller

To launch the project, use the following command:

```bash
# Launch the project
ros2 launch drone_control tello_control_launch.py
```

### QR Code Triggered Behaviors

- **QR Code Detection**

This package allows the control of a drone based on the detection of specific QR codes. The drone performs different actions depending on the QR code it detects. 

The following QR codes are available to trigger different scenarios:

- **start**: Start the *travelling_mode*. In this case, the drone starts moving to the right at a slow speed.

![img](/images/start.png)

- **stop**: Stops the drone immediately, regardless of the current scenario.

![img](/images/stop.png)

- **finish**: Ends the *travelling_mode* and stops the drone.
  
![img](/images/finish.png)

- **drop_area**: Nothing implemented.
  
![img](/images/drop_area.png)

- **blue_block**: Nothing implemented.
  
![img](/images/blue_block.png)

- **red_block**: Nothing implemented.
  
![img](/images/red_block.png)

## ⚠️Safety Precautions

- **Operational Safety**
  - Please fly in a safe, open area away from obstacles and people.
  - Use gloves and glasses

- **Battery Management**
  - Ensure the drone’s battery is fully charged before each session.


## To Improve

Switching from *Monitoring* mode to *Travelling* mode does not work correctly, as the drone continues to rotate on itself. However, *Travelling* mode works well when activated independently. You can see it in action in the following video.

[Travelling mode](/images/mode_travelling.mp4)
