# Charger Return Service

## 概述

充电桩回充服务包是一个基于ROS2的自主导航系统，用于实现机器人的自动充电功能。该包集成了激光雷达充电桩检测、位置服务和导航功能，实现完整的充电桩回充流程。

## 功能特性

- **充电桩检测**: 使用激光雷达数据检测充电桩位置
- **位置服务**: 提供充电桩位置查询和管理服务
- **自主导航**: 基于Nav2的路径规划和导航功能
- **回充控制**: 完整的充电桩对接和充电流程控制

## 依赖包

- `rclpy`: ROS2 Python客户端库
- `geometry_msgs`: 几何消息类型
- `nav2_simple_commander`: Nav2简单导航接口
- `std_msgs`: 标准消息类型
- `laser_charger_detector`: 激光雷达充电桩检测包
- `service_position`: 位置服务包

## 使用方法

### 1. 编译包
```bash
cd /home/robot/ws_navigation2
colcon build --packages-select charger_return_service
```

### 2. 运行服务
```bash
# 启动充电桩回充服务
ros2 run charger_return_service charger_return_node
```

### 3. 调用服务
```bash
# 请求回充服务
ros2 service call /charger_return_service/return_to_charger std_srvs/srv/Trigger
```

## 节点说明

### charger_return_node
主要的充电桩回充控制节点，负责：
- 接收回充请求
- 调用充电桩检测服务
- 执行导航到充电桩
- 控制充电流程

## 服务接口

### 输入服务
- `/charger_detection/detect`: 充电桩检测服务
- `/position_service/get_position`: 位置查询服务
- `/position_service/set_position`: 位置设置服务

### 输出服务
- `/charger_return_service/return_to_charger`: 回充请求服务

## 话题接口

### 订阅话题
- `/scan`: 激光雷达数据
- `/amcl_pose`: 机器人位置信息

### 发布话题
- `/cmd_vel`: 机器人速度控制
- `/charger_status`: 充电状态信息

## 配置参数

- `detection_timeout`: 检测超时时间（秒）
- `navigation_timeout`: 导航超时时间（秒）
- `charging_timeout`: 充电超时时间（秒）
- `max_retry_attempts`: 最大重试次数

## 开发计划

- [ ] 实现充电桩检测集成
- [ ] 实现位置服务集成
- [ ] 实现导航功能
- [ ] 实现充电控制逻辑
- [ ] 添加错误处理和恢复机制
- [ ] 添加配置参数支持
- [ ] 添加日志和监控功能 