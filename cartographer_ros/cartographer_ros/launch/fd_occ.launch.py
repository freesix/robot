from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

declareTopic = DeclareLaunchArgument('topic', default_value='map')

def generate_launch_description(): 
    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05},
            {'occupancy_grid_topic': LaunchConfiguration('topic')}],
        )

    return LaunchDescription([
        declareTopic,
        cartographer_occupancy_grid_node,
    ])
