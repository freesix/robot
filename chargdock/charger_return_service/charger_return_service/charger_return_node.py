#!/usr/bin/env python3
# Copyright 2024 Robot
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from re import S
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from service_position_msgs.srv import GetPosition
from service_position_msgs.srv import SetPosition
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration



class ChargerReturnNode(Node):

    def __init__(self):
        super().__init__('charger_return_node')
        
        self._init_service_clients()
        self.navigator = BasicNavigator()
        # 初始化x, y
        self.x = None
        self.y = None

        self.orientation_x = None
        self.orientation_y = None
        self.orientation_z = None
        self.orientation_w = None
        
        self.status_flag = 0
        self.idle_stage = 0  # 添加空闲状态的阶段标记
        self.need_init_position = 0  # 添加位置初始化标志位
        
        # 创建定时器，延迟执行导航任务
        self.timer = self.create_timer(3.5, self.state_machine)
        self.status_srv = self.create_service(SetPosition, 'charger_return_status', self.status_service_callback)
        # 初始化时获取充电桩位置
        self._init_position()
        self.get_logger().info("充电桩回充服务节点初始化完成")
        
    def _init_service_clients(self):  
        self.start_task_client = self.create_client(
            Trigger, '/start_task')
        self.sleep_node_client = self.create_client(
            Trigger, '/sleep_node')
        self.get_position_client = self.create_client(
            GetPosition, '/get_position')
        self.start_detection_client = self.create_client(
            Trigger, '/start_detection')
        self.stop_detection_client = self.create_client(
            Trigger, '/stop_detection')
        self.set_position_client = self.create_client(
            SetPosition, '/set_position')
        self.get_status_client = self.create_client(
            Trigger, '/get_status')
            
        # 等待服务可用
        self._wait_for_services()
        
    def _wait_for_services(self):
        services_to_wait = [
            ('/start_task', self.start_task_client),
            ('/sleep_node', self.sleep_node_client),
            ('/get_position', self.get_position_client),
            ('/get_status', self.get_status_client)
        ]
        
        for service_name, client in services_to_wait:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'等待服务 {service_name} 可用...')
                
        self.get_logger().info("所有服务已就绪")
        
    def start_chassis_task(self):
        request = Trigger.Request()
        future = self.start_task_client.call_async(request)
        self.get_logger().info("启动底盘控制任务")
        return future
        
    def sleep_chassis_node(self):
        request = Trigger.Request()
        future = self.sleep_node_client.call_async(request)
        self.get_logger().info("休眠底盘控制节点")
        return future
        
    def get_charger_position(self):
        request = GetPosition.Request()
        future = self.get_position_client.call_async(request)
        self.get_logger().info("获取充电桩位置")
        return future

    def start_detection(self):
        request = Trigger.Request()
        future = self.start_detection_client.call_async(request)
        self.get_logger().info("启动凹槽检测")
        return future

    def stop_detection(self):
        request = Trigger.Request()
        future = self.stop_detection_client.call_async(request)
        self.get_logger().info("停止凹槽检测")
        return future

    def set_position(self):
        request = SetPosition.Request()
        # 不设置任何参数，让服务自动获取当前检测到的位置
        future = self.set_position_client.call_async(request)
        self.get_logger().info("设置充电桩位置")
        return future

    def get_status(self):
        request = Trigger.Request()
        future = self.get_status_client.call_async(request)
        self.get_logger().info("查询对齐任务状态")
        return future

    def wait_for_alignment_completion(self, timeout_seconds=120):
        start_time = self.get_clock().now()
        max_wait_time = Duration(seconds=timeout_seconds)
        
        while True:
            # 检查是否超时
            if self.get_clock().now() - start_time > max_wait_time:
                self.get_logger().warn(f"等待对齐任务完成超时 ({timeout_seconds}秒)")
                return False
            
            # 查询对齐任务状态
            future = self.get_status()
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                if response is not None and response.success:
                    self.get_logger().info("对齐任务已完成")
                    return True
                else:
                    self.get_logger().info(f"对齐任务进行中: {response.message if response else '未知状态'}")
            else:
                self.get_logger().warn("状态查询超时，继续等待")
            
            # 等待一段时间后再次查询
            self.get_clock().sleep_for(Duration(seconds=2))

    def status_service_callback(self, request, response):
        try:
            flag = int(request.x)
            self.status_flag = flag
            response.success = True
            response.message = f"状态已更新为{flag}"
        except Exception as e:
            response.success = False
            response.message = f"解析状态失败: {str(e)}"
        return response

    def _init_position(self):
        future = self.get_charger_position()
        rclpy.spin_until_future_complete(self, future) 
        try:
            response = future.result()
            if response is not None and hasattr(response, 'success') and response.success:
                self.x = getattr(response, 'x', None)
                self.y = getattr(response, 'y', None)
                self.orientation_x = getattr(response, 'orientation_x', None)
                self.orientation_y = getattr(response, 'orientation_y', None)
                self.orientation_z = getattr(response, 'orientation_z', None)
                self.orientation_w = getattr(response, 'orientation_w', None)
                self.get_logger().info(f"获取到充电桩位置: x={self.x}, y={self.y}")
                self.get_logger().info(f"获取到充电桩姿态: ox={self.orientation_x}, oy={self.orientation_y}, oz={self.orientation_z}, ow={self.orientation_w}")
            else:
                self.get_logger().error("获取充电桩位置失败")
                if response is not None:
                    self.get_logger().error(f"失败原因: {getattr(response, 'message', '未知')}")
        except Exception as e:
            self.get_logger().error(f"获取充电桩位置时发生错误: {str(e)}")
            self.status_flag = 0

    def navigate_to_charger(self):
        if self.x is None or self.y is None:
            self.get_logger().error("充电桩位置未初始化，无法导航")
            return False
        
        try:
            # 等待导航系统激活
            self.get_logger().info("等待导航系统激活...")
            #navigator.waitUntilNav2Active(localizer='cartographer_node')
            navigator = self.navigator 
            self.get_logger().info("导航系统已激活")
            
            # 创建目标位姿
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = self.x
            goal_pose.pose.position.y = self.y
            
            # 使用初始化中获得的四元数
            if (self.orientation_x is not None and self.orientation_y is not None and 
                self.orientation_z is not None and self.orientation_w is not None):
                goal_pose.pose.orientation.x = self.orientation_x
                goal_pose.pose.orientation.y = self.orientation_y
                goal_pose.pose.orientation.z = self.orientation_z
                goal_pose.pose.orientation.w = self.orientation_w
                self.get_logger().info(f"使用充电桩姿态: x={self.orientation_x}, y={self.orientation_y}, z={self.orientation_z}, w={self.orientation_w}")
            else:
                goal_pose.pose.orientation.w = 1.0  # 默认朝向
                self.get_logger().warn("未获取到充电桩姿态，使用默认朝向")
            
            self.get_logger().info(f"开始导航到充电桩位置: x={self.x}, y={self.y}")
            
            # 执行导航
            navigator.goToPose(goal_pose)
            
            # 等待导航完成
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                    # 如果导航时间超过10分钟，取消任务
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.get_logger().warn("导航超时，取消任务")
                    navigator.cancelTask()
                    break
            
            # 获取导航结果
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("成功到达充电桩位置！")
                return True
            elif result == TaskResult.CANCELED:
                self.get_logger().warn("导航任务被取消")
                return False
            elif result == TaskResult.FAILED:
                self.get_logger().error("导航任务失败")
                return False
            else:
                self.get_logger().error("导航任务返回无效状态")
                return False
                
        except Exception as e:
            self.get_logger().error(f"导航过程中发生错误: {str(e)}")
            return False


    def execute_charging_task(self):
        # 启动检测服务
        self.start_detection()
        # 检查是否已获取充电桩位置
        if self.x is None or self.y is None:
            self.get_logger().error("未能获取充电桩位置，无法执行导航任务")
            self.status_flag = 0  # 重置状态标志
            return
        
        self.get_logger().info("开始执行充电桩回充任务...")
        
        try:
            # 导航到充电桩位置
            self.get_logger().info("开始导航到充电桩...")
            navigation_success = self.navigate_to_charger()
            
            if navigation_success:
                self.get_logger().info("成功到达充电桩附近，开始精确对齐...")
                self.start_chassis_task()
                
                # 等待对齐任务完成
                self.get_logger().info("等待精确对齐任务完成...")
                alignment_success = self.wait_for_alignment_completion()
                
                if alignment_success:
                    self.get_logger().info("精确对齐完成，开始充电...")
                else:
                    self.get_logger().warn("对齐任务未完成，但继续执行后续步骤")
            else:
                self.get_logger().error("导航失败，无法到达充电桩")
            
            # 休眠底盘控制节点
            self.get_logger().info("休眠底盘控制节点...")
            self.sleep_chassis_node()
            self.stop_detection()
            self.get_logger().info("充电桩回充任务完成")
            
            # 任务完成后重置状态标志，允许下次执行
            self.status_flag = 0
            
        except Exception as e:
            self.get_logger().error(f"执行充电任务时发生错误: {str(e)}")
            # 发生错误时也重置状态标志
            self.status_flag = 0

    def state_machine(self):
        if self.status_flag == 0:
            # 等待状态，不做任何操作
            return
        elif self.status_flag == 1:
            # 执行充电任务
            self.execute_charging_task()
        elif self.status_flag == 2:
            # 执行其他任务
            self.execute_other_task()
        elif self.status_flag == 3:
            # 执行空闲任务
            self.execute_idle()
        else:
            self.get_logger().warn(f"未知状态标志: {self.status_flag}")
            # 重置为等待状态
            self.status_flag = 0

    def execute_idle(self):
        if self.idle_stage == 0:
            try:
                # 启动凹槽检测服务
                self.get_logger().info("启动凹槽检测服务...")
                self.start_detection()
                self.get_logger().info("凹槽检测服务启动成功，等待检测充电桩...")
                self.idle_stage = 1  # 进入下一阶段
               
            except Exception as e:
                self.get_logger().error(f"启动检测服务时发生错误: {str(e)}")
                self.status_flag = 0  
                
        elif self.idle_stage == 1:
            # 第二次进入：保存充电桩位置
            self.get_logger().info("空闲状态 - 阶段2：保存充电桩位置")
            
            try:
                # 启动位置设置服务，保存识别到的充电桩位置和朝向
                self.get_logger().info("启动位置设置服务...")
                set_future = self.set_position()
                set_response = set_future.result()
                
                if set_response and set_response.success:
                    self.get_logger().info("充电桩位置和朝向保存成功")
                    self.get_logger().info(f"服务结果: {set_response.message}")
                else:
                    self.get_logger().error("充电桩位置和朝向保存失败")
                
                # 关闭凹槽检测服务
                self.get_logger().info("关闭凹槽检测服务...")
                self.stop_detection()  
                self.get_logger().info("凹槽检测服务关闭成功")
                # 完成所有步骤，重置状态
                self.get_logger().info("空闲状态任务完成，重置为等待状态")
                self.status_flag = 0  # 重置为等待状态
                self.idle_stage = 0   # 重置阶段
                self.need_init_position = 1  # 设置位置初始化标志位
                
            except Exception as e:
                self.get_logger().error(f"保存位置时发生错误: {str(e)}")
                self.status_flag = 0  # 重置状态
                self.idle_stage = 0   # 重置阶段

    def execute_other_task(self):
        self.get_logger().info("执行其它任务")
        # 任务完成后重置状态标志，允许下次执行
        self.status_flag = 0


def main(args=None):
    rclpy.init(args=args)
    
    node = ChargerReturnNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # 检查是否需要初始化位置
            if node.need_init_position == 1:
                node.get_logger().info("检测到位置初始化标志，开始获取充电桩位置...")
                node._init_position()
                node.need_init_position = 0  # 重置标志位
                node.get_logger().info("位置初始化完成")
                
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号，正在退出...")
    except Exception as e:
        node.get_logger().error(f"执行过程中发生错误: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 