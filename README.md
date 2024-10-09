<img src="/images/Humble.png" align="right" width="300" alt="header pic"/>

# Tello Drone with ROS2

![ROS2](https://img.shields.io/badge/ros2-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)


## Table of Contents

- [Tello Drone with ROS2](#tello-drone-with-ros2)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Objectives](#objectives)
  - [Presentation of the Drone](#presentation-of-the-drone)
    - [Trajectory Control](#trajectory-control)
  - [Mission](#mission)
    - [Scenarios to Implement](#scenarios-to-implement)
    - [Behavioral Modes](#behavioral-modes)
  - [Getting Started](#getting-started)
    - [Prerequisites](#prerequisites)
    - [Installation Steps](#installation-steps)
  - [Implementation Guide](#implementation-guide)
    - [Manual Control with Joystick](#manual-control-with-joystick)
    - [QR Code Triggered Behaviors](#qr-code-triggered-behaviors)
    - [Follower Mode](#follower-mode)
    - [Cinema Mode](#cinema-mode)
  - [Safety Precautions](#safety-precautions)
    - [Scenarios to Implement](#scenarios-to-implement-1)
    - [Behavioral Modes](#behavioral-modes-1)


## Introduction

Welcome to my **Tello Drone with ROS2** project! In this master project, I’ll be exploring the capabilities of [ROS2 Humble](https://docs.ros.org/en/humble/index.html) (Robot Operating System 2) using the DJI [Tello EDU](https://www.ryzerobotics.com/tello-edu) drone.

## Objectives

- **Drone Integration**
- **Implement Control Scenarios**
- **Behavioral Programming**

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

1. **Install ROS2**
   - Follow the official ROS2 [installation guide](https://docs.ros.org/en/foxy/Installation.html) for my operating system.

2. **Set Up the Tello ROS2 Package**

3. **Install Dependencies**


4. **Connect to the Drone**
   - Power on the Tello EDU drone.
   - Connect my PC to the drone’s Wi-Fi network.
   - Check the connection by pinging the drone’s IP address.

## Implementation Guide

### Manual Control with Joystick

- **Configure the Joystick**


- **Develop ROS2 Nodes**


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


- **Programming Behaviors**


### Follower Mode


### Cinema Mode


## Safety Precautions

- **Operational Safety**
  - Please fly in a safe, open area away from obstacles and people.
  - Use gloves and glasses

- **Battery Management**
  - Ensure the drone’s battery is fully charged before each session.




### Scenarios to Implement

1. **Manual Control with Joystick**
   - Control the drone manually using a joystick connected to your PC for precise maneuvers in any direction.

2. **QR Code Triggered Behaviors**
   - Program the drone to execute predefined actions when a QR code is detected, such as starting, stopping, or triggering other scenarios like surveillance or item drops.

### Behavioral Modes

- **Follower Mode**
  - The drone will follow the detected QR code, adjusting its position in real-time to maintain a relative distance or specific alignment to the code.

- **Monitoring Mode**
  - The drone will turn on itself continuously to simulate a monitoring scenario, capturing its surroundings without moving away from its location.

- **Manual Mode**
  - Full manual control of the drone using a joystick, allowing for precise movements and adjustments during flight.