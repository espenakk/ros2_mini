# Qube Bringup Package

## Overview
The `controlled_qube` package is designed to bring up the Qube robot in a ROS 2 environment. It includes launch files, URDF descriptions, and configurations necessary to start and visualize the Qube robot.

## Controlled Qube

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

## Features
- Launch configuration for the Qube robot
- URDF description for the Qube robot
- Integration with `qube_driver` and `qube_controller` packages
- RViz configuration for visualizing the robot

## Usage
To launch the Qube robot, use the following command:
```sh
ros2 launch controlled_qube bringup.launch.py
```

## Launch Arguments
- `baud_rate`: Baud rate for communication with the Qube device (default: '115200').
- `device`: Path to the device (default: '/dev/ttyUSB0').
- `simulation`: Sets the system in simulation mode if TRUE (default: 'false').

## Nodes
- `robot_state_publisher`: Publishes the robot state using the URDF description.
- `rviz`: Launches RViz with a specified configuration file.
- `qube_controller`: Node for controlling the Qube device.

## Included Launch Files
- `qube_driver_launch`: Includes the launch file for the `qube_driver` package.

## License
This project is licensed under the MIT License.