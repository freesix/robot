from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from  ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    web_control_dir = get_package_share_directory('web_control') 
    return LaunchDescription([
        Node(
            package='web_control',
            executable='initial_pose',
            name='initial_pose',
            output='screen',
            parameters=[
                {'configuration_directory': FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files'},
                {'configuration_basename': 'fd.lua'},
                {'json_file': os.path.join(web_control_dir, 'launch', 'last_pose.json')}, 
            ]
        )
    ])