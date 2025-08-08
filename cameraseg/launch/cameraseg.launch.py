from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description(): 
    left_sub = LaunchConfiguration('left_sub')
    left_pub = LaunchConfiguration('left_pub')
    right_sub = LaunchConfiguration('right_sub')
    right_pub = LaunchConfiguration('right_pub')

    left_sub_cmd = DeclareLaunchArgument('left_sub', default_value='/camera_left/depth/color/points')
    left_pub_cmd = DeclareLaunchArgument('left_pub', default_value='/camera_left/depth/points')
    right_sub_cmd = DeclareLaunchArgument('right_sub', default_value='/camera_right/depth/color/points')
    right_pub_cmd = DeclareLaunchArgument('right_pub', default_value='/camera_right/depth/points')

    left_node = Node(
        package='cameraseg',
        executable='cameraseg',
        parameters=[{
            'topic' : left_sub,
            'pub' : left_pub     
        }]    
    )   

    right_node = Node(
        package='cameraseg',
        executable='cameraseg',
        parameters=[{
            'topic' : right_sub,
            'pub' : right_pub 
        }]     
    )

    return LaunchDescription([
        left_sub_cmd,
        left_pub_cmd,
        right_sub_cmd,
        right_pub_cmd,
        left_node,
        right_node      
    ]) 
    