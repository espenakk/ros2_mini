from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', [os.path.join(get_package_share_directory('qube_description'), 'config', 'rviz_config.rviz')]]
    )
   
    gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    xacro_file = os.path.join(get_package_share_directory('qube_description'),'urdf','qube.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}] # adds a URDF to the robot description
    )

    return LaunchDescription([
      node_robot_state_publisher,      
      rviz,
      gui
      ])