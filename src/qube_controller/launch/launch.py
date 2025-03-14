from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
   kp_launch_arg = DeclareLaunchArgument('kp', default_value=TextSubstitution(text='5.0'))
   ki_launch_arg = DeclareLaunchArgument('ki', default_value=TextSubstitution(text='0.001'))
   kd_launch_arg = DeclareLaunchArgument('kd', default_value=TextSubstitution(text='0.5'))
   
   config = os.path.join(
      get_package_share_directory('pid_controller'),
      'launch',
      'config',
      'parameters.yaml'
   )

   return LaunchDescription([
      kp_launch_arg,
      ki_launch_arg,
      kd_launch_arg,
      Node(
         package='pid_controller',
         executable='pid_controller_node',
         name='pid_controller',
         parameters=[{
            'kp': LaunchConfiguration('kp'),
            'ki': LaunchConfiguration('ki'),
            'kd': LaunchConfiguration('kd'),
         }]
      ),
      Node(
         package='joint_simulator',
         executable='joint_simulator_node',
         #namespace='joint_simulator',
         name='joint_simulator',
         parameters=[config]
      ),
      Node(
         package='pid_controller',
         executable='reference_input_node',
         name='reference_input'
      )
   ])