# Qube Controller

## Description
The Qube Controller package provides ROS2 nodes that manage a system using a PID controller. It allows for dynamic configuration of PID gains and reference values through ROS2 parameters.

## Running the Node
To launch the PID Controller node, execute the following:
```
ros2 run qube_controller qube_controller_node
```

### Changing Qube Controller Parameters
The Qube controller node allows dynamic parameter updates. Use the following commands to change the PID parameters or the reference value:

1. Update the proportional gain (`kp`):
   ```bash
   ros2 param set /qube_controller kp 10.0
   ```

2. Update the integral gain (`ki`):
   ```bash
   ros2 param set /qube_controller ki 0.001
   ```

3. Update the derivative gain (`kd`):
   ```bash
   ros2 param set /qube_controller kd 20.0
   ```

4. Update the reference value (`ref`):
   ```bash
   ros2 param set /qube_controller ref 1.57
   ```

---