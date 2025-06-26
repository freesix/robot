from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 声明一个字符串参数 "topic_name"
        DeclareLaunchArgument(
            'topic',
            default_value='/imu/data',
            description='The topic name to publish IMU data'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200', 
        ),
        DeclareLaunchArgument(
            'dev',
            default_value='/dev/ttyzfyimu', 
        ),
        DeclareLaunchArgument(
            'imu_link',
            default_value='imu_link',
        ),
        
        # 启动 imu_publisher 节点，并传入 topic_name 参数
        Node(
            package='zfy_imu_node',
            executable='zfy_imu_node',
            # name='imu_publisher_node',
            output='screen',
            parameters=[{'topic': LaunchConfiguration('topic'),
                        'baud_rate': LaunchConfiguration('baud_rate'),
                        'dev': LaunchConfiguration('dev'),
                        'imu_link': LaunchConfiguration('imu_link')}]
        ),
    ])
