import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('laser_charger_detector')
    
    # 定义启动参数，允许从命令行覆盖
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 创建底盘控制器节点
    chassis_controller_node = Node(
        package='laser_charger_detector',
        executable='chassis_controller',
        name='chassis_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            
            # 路径跟踪阶段PID参数
            'linear_x_pid.p': 0.8,
            'linear_x_pid.i': 0.005,
            'linear_x_pid.d': 0.1,
            'linear_x_pid.i_max': 0.3,
            'linear_x_pid.i_min': -0.15,
            
            'linear_y_pid.p': 0.8,
            'linear_y_pid.i': 0.005,
            'linear_y_pid.d': 0.1,
            'linear_y_pid.i_max': 0.3,
            'linear_y_pid.i_min': -0.15,
            
            'angular_z_pid.p': 0.5,
            'angular_z_pid.i': 0.005,
            'angular_z_pid.d': 0.01,
            'angular_z_pid.i_max': 0.25,
            'angular_z_pid.i_min': -0.25,
            
            # 精细调整阶段PID参数
            'fine_tuning/linear_x_pid.p': 0.5,
            'fine_tuning/linear_x_pid.i': 0.01,
            'fine_tuning/linear_x_pid.d': 0.15,
            'fine_tuning/linear_x_pid.i_max': 0.1,
            'fine_tuning/linear_x_pid.i_min': -0.1,
            
            'fine_tuning/angular_z_pid.p': 0.6,       # 进一步提高比例增益，加快收敛
            'fine_tuning/angular_z_pid.i': 0.005,     # 进一步降低积分增益，减少震荡
            'fine_tuning/angular_z_pid.d': 0.12,      # 增加微分增益，提供更强阻尼
            'fine_tuning/angular_z_pid.i_max': 0.3,   # 降低积分限幅，避免积分饱和
            'fine_tuning/angular_z_pid.i_min': -0.3,
        }],
    )
    groove_detector_node = Node(
        package='laser_charger_detector',
        executable='groove_detector',
        name='groove_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    
    # 创建RViz可视化节点  
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
            
        # 启动节点
        chassis_controller_node,
        groove_detector_node,
        #rviz_node,
    ]) 