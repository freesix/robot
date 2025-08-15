import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径（仅用于调试信息）
    try:
        charger_return_pkg_dir = get_package_share_directory('charger_return_service')
        service_position_pkg_dir = get_package_share_directory('service_position')
        laser_charger_detector_pkg_dir = get_package_share_directory('laser_charger_detector')
        
        # 打印调试信息
        print(f"找到的包路径:")
        print(f"  charger_return_service: {charger_return_pkg_dir}")
        print(f"  service_position: {service_position_pkg_dir}")
        print(f"  laser_charger_detector: {laser_charger_detector_pkg_dir}")
    except Exception as e:
        print(f"警告：无法获取包路径信息 - {e}")
        print("继续启动节点...")
    
    # 定义启动参数，允许从命令行覆盖
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 创建charger_return_service节点
    charger_return_node = Node(
        package='charger_return_service',
        executable='charger_return_node',
        name='charger_return_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    
    # 创建service_position节点
    service_position_node = Node(
        package='service_position',
        executable='service_position_node',
        name='service_position_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    
    # 直接启动chassis_controller节点（而不是包含launch文件）
    chassis_controller_node = Node(
        package='laser_charger_detector',
        executable='chassis_controller',
        name='chassis_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            
            # 路径跟踪阶段PID参数
            'linear_x_pid.p': 1.0,
            'linear_x_pid.i': 0.005,
            'linear_x_pid.d': 0.15,
            'linear_x_pid.i_max': 0.3,
            'linear_x_pid.i_min': -0.15,
            
            'linear_y_pid.p': 1.0,
            'linear_y_pid.i': 0.005,
            'linear_y_pid.d': 0.15,
            'linear_y_pid.i_max': 0.3,
            'linear_y_pid.i_min': -0.15,
            
            'angular_z_pid.p': 1.5,
            'angular_z_pid.i': 0.005,
            'angular_z_pid.d': 0.3,
            'angular_z_pid.i_max': 0.25,
            'angular_z_pid.i_min': -0.25,
            
            # 精细调整阶段PID参数 - 优化收敛速度
            'fine_tuning/angular_z_pid.p': 0.6,       
            'fine_tuning/angular_z_pid.i': 0.005,     
            'fine_tuning/angular_z_pid.d': 0.12,      
            'fine_tuning/angular_z_pid.i_max': 0.3,  
            'fine_tuning/angular_z_pid.i_min': -0.3,
        }],
    )
    
    # 直接启动groove_detector节点
    groove_detector_node = Node(
        package='laser_charger_detector',
        executable='groove_detector',
        name='groove_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    
    # 创建RViz可视化节点（可选）
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', os.path.join(charger_return_pkg_dir, 'config', 'charger_return.rviz')],
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #     }],
    # )
    
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
            
        # 启动节点
        charger_return_node,
        service_position_node,
        chassis_controller_node,
        groove_detector_node,
        # rviz_node,  # 取消注释以启用RViz可视化
    ]) 