# Qube Bringup Package

## Overview
The `controlled_qube` package is designed to bring up the Qube robot in a ROS 2 environment. It includes launch files, URDF descriptions, and configurations necessary to start and visualize the Qube robot.

## Controlled qube urdf

This URDF file is used to describe the robot model for the "qube" robot. It includes necessary macros and configurations for the robot's description and control.

### Elements:
- `<robot>`: The root element defining the robot with the name "qube".
- `<xacro:include>`: Includes external xacro files for robot description and control configuration.
  - `qube_description`: Contains the macro definitions for the qube robot.
  - `qube_driver`: Contains the ROS 2 control configurations for the qube driver.
- `<xacro:arg>`: Defines arguments that can be passed to the xacro file.
  - `baud_rate`: The communication baud rate (default: 115200).
  - `device`: The device file for the serial connection (default: /dev/ttyUSB0).
  - `simulation`: A flag to indicate if the robot is in simulation mode (default: false).
- `<link>`: Defines a link named "world" which serves as the fixed reference frame.
- `<joint>`: Defines a fixed joint named "base_joint" connecting the "world" link to the "base_link".
- `<xacro:qube>`: Macro call to include the qube robot description.
- `<xacro:qube_driver_ros2_control>`: Macro call to include the ROS 2 control configuration for the qube driver.
  - `prefix`: Prefix for the names (empty in this case).
  - `name`: Name of the driver (qube_driver).
  - `baud_rate`: Communication baud rate.
  - `device`: Device file for the serial connection.
  - `simulation`: Simulation mode flag.

## Run
To launch the Qube robot, use the following command:
```sh
ros2 launch controlled_qube bringup.launch.py
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