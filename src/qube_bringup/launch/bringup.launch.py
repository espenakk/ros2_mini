from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', [os.path.join(get_package_share_directory('qube_bringup'), 'config', 'rviz_config.rviz')]]
    )

    xacro_file = os.path.join(get_package_share_directory('qube_bringup'),'urdf','controlled_qube.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}] # adds a URDF to the robot description
    )

    #TODO We probably need to remove this, only added cause we were missing a joint_state_publisher
    # Add joint_state_publisher to provide joint states when not from hardware
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
    
    # Include qube_driver.launch.py from the qube_driver package
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('qube_driver'), 'launch', 'qube_driver.launch.py')
        )
    )

    return LaunchDescription([
      node_robot_state_publisher,
      joint_state_publisher,
      rviz,
      qube_driver_launch
      ])