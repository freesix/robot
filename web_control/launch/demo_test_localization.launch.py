from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from  ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    lakibeam_dir = get_package_share_directory('lakibeam1')
    serial_imu_dir = get_package_share_directory('serial_imu')
    serial_imu_old_dir = get_package_share_directory('serial_imu_old')
    motor_dir = get_package_share_directory('motor_drive')
    cartographer_ros_dir = get_package_share_directory('cartographer_ros')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                lakibeam_dir, 'launch', 'lakibeam1_scan.launch.py'))        
        ),
        # IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(os.path.join(
                # serial_imu_dir, 'launch', 'serial_imu.launch.py')) 
        # ),    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                serial_imu_old_dir, 'launch', 'serial_imu_old.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                motor_dir, 'launch', 'robot_motor_base.launch.py')) 
        ), 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                cartographer_ros_dir, 'launch', 'demo_fd_localization.launch.py'))   
        ), 
    ])