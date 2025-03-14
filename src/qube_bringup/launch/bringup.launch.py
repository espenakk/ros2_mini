from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Include qube_driver.launch.py from the qube_driver package
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('qube_driver'), 'launch', 'qube_driver.launch.py')
        )
    )

    # Process controlled_qube.urdf.xacro for robot_state_publisher
    xacro_file = os.path.join(get_package_share_directory('qube_bringup'), 'urdf', 'controlled_qube.urdf.xacro')
    # You can pass any necessary xacro arguments as a dict (if needed)
    robot_description = xacro.process_file(xacro_file).toxml()

    # Define rviz2 node (without a custom config, can be added if available)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
        # ,arguments=['-d', '<path-to-rviz-config>']  # Optionally add a config file
    )

    # Define robot_state_publisher node to publish the robot_description
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        qube_driver_launch,
        rviz_node,
        rsp_node,
    ])
