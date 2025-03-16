"""
Generate the launch description for the qube_bringup package.
This function sets up the launch configuration for the qube_bringup package, including
declaring launch arguments, setting up nodes, and including other launch files.
Launch Arguments:
- baud_rate: Baud rate for communication with the qube device (default: '115200').
- device: Path to the device (default: '/dev/ttyUSB0').
- simulation: Sets the system in simulation mode if TRUE (default: 'false').
Nodes:
- robot_state_publisher: Publishes the robot state using the URDF description.
- rviz: Launches RViz with a specified configuration file.
- qube_controller: Node for controlling the qube device.
Included Launch Files:
- qube_driver_launch: Includes the launch file for the qube_driver package.
Returns:
LaunchDescription: The launch description containing all the nodes and configurations.
"""

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    baud_rate_arg = DeclareLaunchArgument('baud_rate', default_value=TextSubstitution(text='115200'),
        description='Baud rate for communication with qube device')
    device_arg = DeclareLaunchArgument('device', default_value=TextSubstitution(text='/dev/ttyUSB0'),
        description='Path to device (/dev/ttyUSB0/)')
    simulation_arg = DeclareLaunchArgument('simulation', default_value=TextSubstitution(text='false'),
        description='Sets the system in simulation mode if TRUE')
    
    robot_description_content = ParameterValue(Command(['xacro ',PathJoinSubstitution(
        [FindPackageShare('qube_bringup'),'urdf','controlled_qube.urdf.xacro']),
            ' ', 'baud_rate:=', LaunchConfiguration('baud_rate'),
            ' ', 'device:=', LaunchConfiguration('device'),
            ' ', 'simulation:=', LaunchConfiguration('simulation')]),
            value_type=str
            )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}] # adds a URDF to the robot description
        )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',  PathJoinSubstitution([FindPackageShare('qube_bringup'),'config','rviz_config.rviz'])]
        )

    qube_driver_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('qube_driver'),'launch','qube_driver.launch.py']))
        )

    qube_controller = Node(
        package='qube_controller',
        executable='qube_controller_node',
        name='qube_controller',
        output='screen',
        )
    
    return LaunchDescription([
        baud_rate_arg,
        device_arg,
        simulation_arg,

        robot_state_publisher,
        rviz,
        qube_controller,

        qube_driver_launch
      ])