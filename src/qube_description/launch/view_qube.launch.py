from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():

    # Set up xacro file and robot description
    robot_description_content = ParameterValue(Command(['xacro ',PathJoinSubstitution([FindPackageShare('qube_description'),'urdf','qube.urdf.xacro'])]),
        value_type=str
    )

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Rviz with config
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('qube_description'),'config','rviz_config.rviz'])]
    )
   
   # Joint state publisher gui
    gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        # Core nodes
        node_robot_state_publisher,      
        rviz,
        gui
      ])


