# Qube Description

This ROS 2 package describes the "qube" robot model using Xacro and URDF files. It includes macros to define a compact model consisting of a base, stator, rotor, and an angle indicator.

## Qube Macro

This file defines a Xacro macro for a simple robot model with a base, stator, rotor, and angle link. The macro is parameterized with a prefix to allow for multiple instances with unique names.

### Properties

- `box_size`: Size of the box representing the stator link.
- `cylinder_length`: Length of the cylinder representing the rotor link.
- `cylinder_radius`: Radius of the cylinder representing the rotor link.
- `pointer_length`: Length of the pointer (same as cylinder_radius).
- `pointer_width`: Width of the pointer.
- `pointer_height`: Height of the pointer.
- `small_box_size`: Dimensions of the small box representing the angle link.

### Macro: `qube`

#### Parameters

- `prefix`: A string to prefix all link and joint names to ensure uniqueness.

#### Links

- `${prefix}base_link`: The base link of the robot.
- `${prefix}stator_link`: The stator link, visualized as a black box.
- `${prefix}rotor_link`: The rotor link, visualized as a red cylinder.
- `${prefix}angle_link`: The angle link, visualized as a white box positioned on top of the rotor.

#### Joints

- `${prefix}stator_joint`: A fixed joint connecting the base link to the stator link.
- `${prefix}motor_joint`: A revolute joint connecting the stator link to the rotor link, allowing rotation around the Z-axis.
- `${prefix}indicator`: A fixed joint connecting the rotor link to the angle link, positioning the angle link on top of the rotor.

## Qube Scene

This URDF file describes the robot model for "qube". It uses the xacro (XML Macros) format to include reusable macros and simplify the URDF definition.

### Elements

- `<robot>`: The root element that defines the robot with the name "qube".
- `<xacro:include>`: Includes the external xacro file `qube.macro.xacro` which contains reusable macros.
- `<link name="world">`: Defines a fixed link representing the world frame.
- `<joint name="base_joint" type="fixed">`: Defines a fixed joint connecting the `world` link to the `base_link`.
    - `<parent link="world"/>`: Specifies the parent link of the joint.
    - `<child link="base_link"/>`: Specifies the child link of the joint.
    - `<origin xyz="0 0 0" rpy="0 0 0"/>`: Specifies the origin of the joint in terms of position (xyz) and orientation (rpy).
- `<xacro:qube prefix="">`: Invokes the `qube` macro defined in the included xacro file, with an empty prefix.

## Run

Launch the visualization:
```
ros2 launch qube_description view_qube.launch.py
```