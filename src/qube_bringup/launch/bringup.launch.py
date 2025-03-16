from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare parameters
    baud_rate_arg = DeclareLaunchArgument('baud_rate', default_value=TextSubstitution(text='115200'),
        description='Baud rate for communication with qube device')
    device_arg = DeclareLaunchArgument('device', default_value=TextSubstitution(text='/dev/ttyUSB0'),
        description='Path to device (/dev/ttyUSB0/)')
    simulation_arg = DeclareLaunchArgument('simulation', default_value=TextSubstitution(text='false'),
        description='Sets the system in simulation mode if TRUE')
    
    # Set up xacro file and robot description
    robot_description_content = ParameterValue(Command(['xacro ',PathJoinSubstitution(
        [FindPackageShare('qube_bringup'),'urdf','controlled_qube.urdf.xacro']),
            ' ', 'baud_rate:=', LaunchConfiguration('baud_rate'),
            ' ', 'device:=', LaunchConfiguration('device'),
            ' ', 'simulation:=', LaunchConfiguration('simulation')]),
            value_type=str
            )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}] # adds a URDF to the robot description
        )

    # Rviz with config
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',  PathJoinSubstitution([FindPackageShare('qube_bringup'),'config','rviz_config.rviz'])]
        )

    # Include qube_driver.launch.py from the qube_driver package
    qube_driver_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('qube_driver'),'launch','qube_driver.launch.py']))
        )

    # PID controller for the qube
    qube_controller = Node(
        package='qube_controller',
        executable='qube_controller_node',
        name='qube_controller',
        output='screen',
        )

    # Qube simulator
    qube_simulator = Node(
        package='qube_simulator',
        executable='qube_simulator_node',
        name='qube_simulator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('simulation'))
        )
    
    return LaunchDescription([
        # Launch arguments
        baud_rate_arg,
        device_arg,
        simulation_arg,

        # Nodes
        robot_state_publisher,
        rviz,
        qube_controller,
        # qube_simulator,

        # Driver launch
        qube_driver_launch
      ])