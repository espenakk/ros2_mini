
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution

"""
Launch file for viewing the Qube robot in RViz.
This launch file performs the following actions:
1. Generates the robot description from the URDF Xacro file.
2. Launches the `robot_state_publisher` node to publish the robot state.
3. Launches the `rviz2` node to visualize the robot in RViz using a specified configuration file.
4. Launches the `joint_state_publisher_gui` node to provide a GUI for joint state publishing.
"""

def generate_launch_description():
    robot_description_content = ParameterValue(Command(['xacro ',PathJoinSubstitution([FindPackageShare('qube_description'),'urdf','qube.urdf.xacro'])]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('qube_description'),'config','rviz_config.rviz'])]
    )
   
    gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,      
        rviz,
        gui
      ])
