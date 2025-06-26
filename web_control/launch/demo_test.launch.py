from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from  ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    lakibeam_dir = get_package_share_directory('lakibeam1')
    serial_imu_dir = get_package_share_directory('serial_imu')
    teleop_joy_dir = get_package_share_directory('teleop_twist_joy')
    # serial_imu_old_dir = get_package_share_directory('serial_imu_old')
    # zfy_imu_dir = get_package_share_directory('zfy_imu_node')
    motor_dir = get_package_share_directory('motor_drive')
    realsense_dir = get_package_share_directory('realsense2_camera')
    cartographer_ros_dir = get_package_share_directory('cartographer_ros')
    cameraseg_dir = get_package_share_directory('cameraseg')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                lakibeam_dir, 'launch', 'lakibeam1_scan.launch.py'))        
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                teleop_joy_dir, 'launch', 'teleop-launch.py'))   
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                serial_imu_dir, 'launch', 'serial_imu.launch.py')) 
        ),    
        # IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(os.path.join(
                # serial_imu_old_dir, 'launch', 'serial_imu_old.launch.py'))
        # ),
        # IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(os.path.join(
                # zfy_imu_dir, 'launch', 'zyf176.launch.py'))    
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                motor_dir, 'launch', 'robot_motor_base.launch.py')) 
        ), 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                cartographer_ros_dir, 'launch', 'demo_fd.launch.py'))   
        ), 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                realsense_dir, 'launch', 'rs_multi_camera_launch.py'))   
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                teleop_joy_dir, 'launch', 'teleop-launch.py'))   
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                cameraseg_dir, 'launch', 'cameraseg.launch.py'))    
        )

    ])