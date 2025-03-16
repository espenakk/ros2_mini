# Qube Controller ROS2 Package

## Overview
The Qube Controller package implements ROS2 nodes for controlling a system using a PID controller. It consists of:
- **PID Controller Node (`qube_controller_node`)**: Subscribes to joint state measurements, calculates control voltages using a PID algorithm, and publishes these voltages.
- **Reference Input Node (`reference_input_node`)**: Prompts the user to input a new reference value and updates the controller via a service.

## Features
- Configurable PID gains and reference value via ROS2 parameters.
- Dynamic parameter updates with a parameter callback.
- Reference validation within the range [-π, π] through a dedicated service.

## Build and Run Instructions

### Prerequisites
- ROS2 installation
- Dependencies: rclcpp, std_msgs, sensor_msgs, qube_controller_msgs

### Build
From the workspace root, run:
```
colcon build --packages-select qube_controller
```

### Run
Launch the PID Controller node:
```
ros2 run qube_controller qube_controller_node
```
Launch the Reference Input node:
```
ros2 run qube_controller reference_input_node
```

## License

This project is licensed under the MIT License.
