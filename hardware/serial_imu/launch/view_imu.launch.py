from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    serial_imu_dir = get_package_share_directory('serial_imu')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                serial_imu_dir, 'launch', 'serial_imu.launch.py'))   
        ),
        Node(
            package='serial_imu',
            executable='serial_imu_view',
            name='imu_view_node',
            output='screen' 
        ),     
    ])
 