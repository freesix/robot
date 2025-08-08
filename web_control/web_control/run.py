import rclpy
import rclpy.logging
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import time
import subprocess

class Run(Node):
    def __init__(self):
        super().__init__("run_app")

        self.map_file = os.path.join(get_package_share_directory('nav2_bringup'), 'maps')
        self.pbstream = os.path.join(self.map_file, 'map.pbstream')
        self.map = os.path.join(self.map_file, 'map.pgm')
        self.map_yaml = os.path.join(self.map_file, 'map.yaml')
        self.mask = os.path.join(self.map_file, 'mask.pgm')
        self.mask_yaml = os.path.join(self.map_file, 'mask.yaml')
        self.processes=[]


    def run_nav(self):
        ret = subprocess.Popen(['ros2', 'launch', 'nav2_bringup', 'fd_robot.launch.py'])
        self.processes.append(ret)
        time.sleep(1)
        if os.path.exists(self.map) and os.path.exists(self.map_yaml) and os.path.exists(self.pbstream):
            self.get_logger().info('map server be launched.')
            ret = subprocess.Popen(['ros2', 'launch', 'nav2_bringup', 'fd_map_server.launch.py'])
            self.processes.append(ret)
            time.sleep(1)
            if os.path.exists(self.mask) and os.path.exists(self.mask_yaml):
                self.get_logger().info('mask file exists, setting...')
                ret = subprocess.Popen(['ros2', 'launch', 'nav2_bringup', 'fd_mask.launch.py'])


    def run_slam(self):
        ret = subprocess.Popen(['ros2', 'launch', 'web_control', 'common.launch.py'])
        self.processes.append(ret) 
        time.sleep(1)
        if os.path.exists(self.pbstream) and os.path.exists(self.map_yaml) and os.path.exists(self.map):
            ret = subprocess.Popen(['ros2', 'launch', 'cartographer_ros', 'fd_slam.launch.py', 'config_name:=fd_localization.lua'])
            self.processes.append(ret)
            time.sleep(1)
            ret = subprocess.Popen(['ros2', 'launch', 'cartographer_ros', 'fd_occ.launch.py', 'topic:=hidden_map'])
            self.processes.append(ret)
            ret = subprocess.Popen(['ros2', 'launch', 'web_control', 'set_initpose.launch.py'])
            self.processes.append(ret)
        else:
            ret = subprocess.Popen(['ros2', 'launch', 'cartographer_ros', 'fd_slam.launch.py', 'config_name:=fd.lua', 'map_path:=""'])
            self.processes.append(ret)
            time.sleep(1)
            ret = subprocess.Popen(['ros2', 'launch', 'cartographer_ros', 'fd_occ.launch.py'])
            self.processes.append(ret)
            time.sleep(1)
    
    def run_http(self):
        ret = subprocess.Popen(['ros2', 'run', 'httpface', 'httpface_node'])
        self.processes.append(ret) 

        



def main(args=None):
    rclpy.init(args=args)
    node = Run()
    node.run_nav()
    node.run_slam()
    node.run_http()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
