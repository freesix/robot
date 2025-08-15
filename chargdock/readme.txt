依赖的pid库https://index.ros.org/p/control_toolbox/#humble
直接apt get安装

两个服务命令：
ros2 service call /charger_return_status service_position_msgs/srv/SetPosition "{x : 1.0}" 识别充电桩并保存map坐标系下的位置和朝向
ros2 service call /charger_return_status service_position_msgs/srv/SetPosition "{x : 3.0}" nav导航到充电桩，并进行对齐操作

charger_return_service是py节点主要是调用nav2的api和其他服务组合使用。
laser_charger_detector是识别充电桩和对齐控制的节点
service_position是保存充电桩位置和提取充电桩位置的节点
service_position_msgs是自定义的服务类型
