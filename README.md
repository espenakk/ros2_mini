# AIS2105 Mini Project

## Overview

This project is a ROS 2-based implementation for controlling a Quanser Qube. It includes multiple packages for describing the robot, controlling it using a PID controller, and bringing up the entire system.

## Packages

### 1. `qube_driver`
This package provides the hardware interface for the Quanser Qube. It includes the necessary configuration files and launch files to interface with the hardware.

- **Launch Files**: 
  - `qube_driver.launch.py`: Launches the ROS 2 control node and spawns the necessary controllers.
- **Configuration Files**: 
  - `qube_driver.ros2_control.xacro`: Defines the ROS 2 control configuration for the Qube.
- **Source Files**: 
  - `qube_driver.cpp`: Implements the hardware interface for the Qube.

### 2. `qube_description`
This package describes the Qube robot model using Xacro and URDF files. It includes macros to define a compact model consisting of a base, stator, rotor, and an angle indicator.

- **URDF Files**: 
  - `qube.urdf.xacro`: Main URDF file that includes the macro.
  - `qube.macro.xacro`: Defines the `qube` macro with configurable parameters.
- **Launch Files**: 
  - `view_qube.launch.py`: Launches RViz to visualize the Qube robot.
- **Configuration Files**: 
  - `rviz_config.rviz`: RViz configuration file.

### 3. `qube_controller`
This package implements ROS 2 nodes for controlling the Qube using a PID controller. It includes nodes for setting the reference value and controlling the Qube.

- **Source Files**: 
  - `qube_controller_node.cpp`: Implements the PID controller node.
  - `reference_input_node.cpp`: Implements the node for setting the reference value.
- **Service Definitions**: 
  - `SetReference.srv`: Service for setting the reference value.

### 4. `qube_controller_msgs`
This package provides custom ROS 2 service definitions for the Qube controller package.

- **Service Definitions**: 
  - `SetReference.srv`: Service for setting the reference value.

### 5. `qube_bringup`
This package is designed to bring up the Qube robot in a ROS 2 environment. It includes launch files, URDF descriptions, and configurations necessary to start and visualize the Qube robot.

- **URDF Files**: 
  - `controlled_qube.urdf.xacro`: Describes the robot model for the Qube.
- **Launch Files**: 
  - `bringup.launch.py`: Launches the entire Qube system.

## Build and Run Instructions

### Prerequisites
- ROS 2 installation
- Dependencies: rclcpp, std_msgs, sensor_msgs, qube_controller_msgs, xacro, robot_state_publisher, rviz2, joint_state_publisher_gui

### Build
From the workspace root, run:
```
colcon build
```

### Source
From the workspace root, run:
```
source install/local_setup.bash
```

### Run
To bring up the entire Qube system, use the following command:
```
ros2 launch qube_bringup bringup.launch.py
```

## License

This project is licensed under the MIT License.