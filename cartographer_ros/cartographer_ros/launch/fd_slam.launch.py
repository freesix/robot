from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os
from ament_index_python.packages import get_package_share_directory

nav2_bringup_dir = get_package_share_directory('nav2_bringup')
declaremap = DeclareLaunchArgument('map_path',
            default_value=os.path.join(nav2_bringup_dir, 'maps', 'map.pbstream'))
declareconfig = DeclareLaunchArgument('config_name', default_value='fd.lua')

def generate_launch_description():
  ## ***** File paths ******
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'fd_base.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    ## ***** Nodes *****
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': False}],
        output = 'screen'
        )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': False},
                      {'configuration_directory': FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files'},
                      {'configuration_basename': LaunchConfiguration('config_name')},
                      {'load_state_filename': LaunchConfiguration('map_path')},
                      {'load_frozen_state': False}],
        arguments = [
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files'],
        remappings = [
            ('scan', '/laser/data'),
            ('imu', '/imu/data')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05},
            {'occupancy_grid_topic': 'map'}],
        )

    return LaunchDescription([
        declaremap,
        declareconfig,
        robot_state_publisher_node,
        cartographer_node,
        # cartographer_occupancy_grid_node,
    ])
