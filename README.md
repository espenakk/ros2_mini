# AIS2105 Mini Project

## Table of Contents
1. [Overview](#overview)
2. [Packages](#packages)
   - [1. `qube_driver`](#1-qube_driver)
   - [2. `qube_description`](#2-qube_description)
   - [3. `qube_controller`](#3-qube_controller)
   - [4. `qube_bringup`](#5-qube_bringup)
3. [Build and Run Instructions](#build-and-run-instructions)
   - [Prerequisites](#prerequisites)
   - [Cloning the Repository](#cloning-the-repository)
   - [Building the Workspace](#building-the-workspace)
   - [Sourcing the Setup](#sourcing-the-workspace)
   - [Running the System](#running-the-system)
   - [Launch File Arguments](#launch-file-arguments)
   - [Modifying Controller Parameters](#modifying-controller-parameters)

## Overview

This project provides a ROS 2-based implementation for controlling a Quanser Qube. It consists of multiple packages designed for the robot's description, control (via a PID controller), and system bring-up.

## Packages

### 1. `qube_driver`
This package provides the hardware interface for the Quanser Qube. It includes configuration files and launch files to interface with the hardware.

- **Launch Files**:  
  - `qube_driver.launch.py`: Launches the ROS 2 control node and spawns the necessary controllers.
  
- **Configuration Files**:  
  - `qube_driver.ros2_control.xacro`: Defines the ROS 2 control configuration for the Qube.

- **Source Files**:  
  - `qube_driver.cpp`: Implements the hardware interface for the Qube.

### 2. `qube_description`
This package describes the Qube robot model using Xacro and URDF files. It includes macros that define a compact model consisting of the base, stator, rotor, and angle indicator.

- **URDF Files**:  
  - `qube.urdf.xacro`: Main URDF file that includes the robot model macro.
  - `qube.macro.xacro`: Defines the `qube` macro with configurable parameters.

- **Launch Files**:  
  - `view_qube.launch.py`: Launches RViz to visualize the Qube robot.

- **Configuration Files**:  
  - `rviz_config.rviz`: RViz configuration file.

### 3. `qube_controller`
This package implements ROS 2 nodes for controlling the Qube using a PID controller. It includes nodes for setting the reference value and controlling the Qube.

- **Source Files**:  
  - `qube_controller_node.cpp`: Implements the PID controller node.

### 4. `qube_bringup`
This package is designed to bring up the Qube robot in a ROS 2 environment. It includes launch files, URDF descriptions, and configurations necessary to start and visualize the Qube robot.

- **URDF Files**:  
  - `controlled_qube.urdf.xacro`: Describes the robot model for the Qube.

- **Launch Files**:  
  - `bringup.launch.py`: Launches the entire Qube system.

## Build and Run Instructions

### Prerequisites
- ROS 2 installation (Jazzy Jalisco)  
  Follow the [ROS 2 Jazzy Jalisco installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
- Required dependencies:  
  - `ros2_control`, `xacro`, `joint-state-broadcaster` and `velocity-controllers`

### Cloning the Repository
To ensure all submodules are cloned, use the following command:
```bash
git clone --recurse-submodules https://github.com/espenakk/ros2_mini.git
```
First update your packages:
```bash
sudo apt update && sudo apt upgrade -y
```
To install the required dependencies, run:
```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-xacro ros-jazzy-joint-state-broadcaster ros-jazzy-velocity-controllers
```

### Building the Workspace
Source ros2 if you have not already done so:
```bash
source /opt/ros/jazzy/setup.bash
```
From the workspace root, run:
```bash
colcon build --symlink-install
```

### Sourcing the workspace
In a new terminal from the workspace root, run:
```bash
source install/local_setup.bash
```

### Running the System
To bring up the entire Qube system, use the following command:
```bash
ros2 launch qube_bringup bringup.launch.py
```

### Launch File Arguments
The `bringup.launch.py` file supports the following arguments:

- `baud_rate`: Baud rate for communication with the Qube device (default: `115200`).
- `device`: Path to the device (default: `/dev/ttyUSB0`).
- `simulation`: Sets the system in simulation mode if `true` (default: `false`).

Example usage with custom arguments:
```bash
ros2 launch qube_bringup bringup.launch.py baud_rate:=9600 device:=/dev/ttyUSB1 simulation:=true
```

### Modifying Controller Parameters
The Qube controller node allows dynamic parameter updates. Use the following commands to change the PID parameters or the reference value:

- Update the proportional gain (`kp`):
  ```bash
  ros2 param set /qube_controller kp 10.0
  ```

- Update the integral gain (`ki`):
  ```bash
  ros2 param set /qube_controller ki 0.001
  ```

- Update the derivative gain (`kd`):
  ```bash
  ros2 param set /qube_controller kd 20.0
  ```

- Update the reference value (`ref`):
  ```bash
  ros2 param set /qube_controller ref 1.57
  ```
