from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 声明一个字符串参数 "topic_name"
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='The topic name to publish IMU data'
        ),
        
        # 启动 imu_publisher 节点，并传入 topic_name 参数
        Node(
            package='serial_imu',
            executable='serial_imu_node',
            # name='imu_publisher_node',
            output='screen',
            parameters=[{'imu_topic': LaunchConfiguration('imu_topic')}]
        ),
    ])
