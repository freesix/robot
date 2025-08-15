import rclpy
import rclpy.logging
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import time
import subprocess
import signal
from web_control_msgs.srv import RunControl 
import sys

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
        self.renewal=[]
        self.maskprocesses=[]
        self.httpproc=[]

        self.srv = self.create_service(
            RunControl,
            'run_control',
            self.callback     
        )

    def callback(self, request, response):
        req_type = request.data

        try:
            if req_type == 1:
                ret = self.start_up()
                response.result = 1 if ret['success'] else -1
                response.messages = str(ret['errors'])
            elif req_type == 2:
                ret = self.kill_all()
                response.result = 1 if ret['success'] else -1
                response.messages = str(ret['errors'])
            elif req_type == 3:
                ret = self.renew()
                response.result = 1 if ret['success'] else -1
                response.messages = str(ret['errors'])
            elif req_type == 4:
                ret = self.remask()
                response.result = 1 if ret['success'] else -1
                response.messages = str(ret['errors'])
            else:
                response.result = 0
                response.messages = f"Unknown request type: {req_type}"
        except Exception as e:
            response.result = 0
            response.messages = f"Error: {e}"

        return response 

    def run_nav(self):
        ret = subprocess.Popen(['ros2', 'launch', 'nav2_bringup', 'fd_robot.launch.py'], preexec_fn=os.setsid)
        self.processes.append(ret)
        time.sleep(1)
        if os.path.exists(self.map) and os.path.exists(self.map_yaml) and os.path.exists(self.pbstream):
            self.get_logger().info('map server be launched.')
            ret = subprocess.Popen(['ros2', 'launch', 'nav2_bringup', 'fd_map_server.launch.py'], preexec_fn=os.setsid)
            self.renewal.append(ret)
            time.sleep(1)
            if os.path.exists(self.mask) and os.path.exists(self.mask_yaml):
                self.get_logger().info('mask file exists, setting...')
                ret = subprocess.Popen(['ros2', 'launch', 'nav2_bringup', 'fd_mask.launch.py'], preexec_fn=os.setsid)
                self.maskprocesses.append(ret)


    def run_slam(self):
        ret = subprocess.Popen(['ros2', 'launch', 'web_control', 'common.launch.py'], preexec_fn=os.setsid)
        self.processes.append(ret) 
        time.sleep(1)
        if os.path.exists(self.pbstream) and os.path.exists(self.map_yaml) and os.path.exists(self.map):
            ret = subprocess.Popen(['ros2', 'launch', 'cartographer_ros', 'fd_slam.launch.py', 'config_name:=fd_localization.lua'], preexec_fn=os.setsid)
            self.renewal.append(ret)
            time.sleep(1)
            ret = subprocess.Popen(['ros2', 'launch', 'cartographer_ros', 'fd_occ.launch.py', 'topic:=hidden_map'], preexec_fn=os.setsid)
            self.renewal.append(ret)
            ret = subprocess.Popen(['ros2', 'launch', 'web_control', 'set_initpose.launch.py'], preexec_fn=os.setsid)
            self.renewal.append(ret)
        else:
            ret = subprocess.Popen(['ros2', 'launch', 'cartographer_ros', 'fd_slam.launch.py', 'config_name:=fd.lua', 'map_path:=""'], preexec_fn=os.setsid)
            self.renewal.append(ret)
            time.sleep(1)
            ret = subprocess.Popen(['ros2', 'launch', 'cartographer_ros', 'fd_occ.launch.py'], preexec_fn=os.setsid)
            self.renewal.append(ret)
            time.sleep(1)
    
    def run_http(self):
        ret = subprocess.Popen(['ros2', 'run', 'httpface', 'httpface_node'], preexec_fn=os.setsid)
        self.httpproc.append(ret) 
        ret = subprocess.Popen(['ros2', 'run', 'web_control', 'map_manager'], preexec_fn=os.setsid)
        self.processes.append(ret) 

    def kill_all(self):
        result = {
            "success": True,
            "errors": []
        }

        if self.processes:
            for p in self.processes:
                try:    
                    pgid = os.getpgid(p.pid)
                    os.killpg(pgid, signal.SIGTERM)
                    p.wait(timeout=3)
                except Exception as e:
                    result["success"] = False
                    result["errors"].append(f"processes PID {p.pid} error: {e}")
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                        result["success"] = True
                    except Exception as e:
                        result['errors'].append(f"force kill error: {e}")

            self.processes.clear()

        if self.maskprocesses:
            for p in self.maskprocesses:
                try:
                    pgid = os.getpgid(p.pid)
                    os.killpg(pgid, signal.SIGTERM)
                    p.wait(timeout=3)
                except Exception as e:
                    result["success"] = False
                    result["errors"].append(f"maskprocesses PID {p.pid} error: {e}")
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                        result["success"] = True
                    except Exception as e:
                        result['errors'].append(f"force kill error: {e}")
                   
            self.maskprocesses.clear()

        if self.renewal:
            for p in self.renewal:
                try:
                    pgid = os.getpgid(p.pid)
                    os.killpg(pgid, signal.SIGTERM)
                    p.wait(timeout=3)
                except Exception as e:
                    result["success"] = False
                    result["errors"].append(f"renewal PID {p.pid} error: {e}")
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                        result["success"] = True
                    except Exception as e:
                        result['errors'].append(f"force kill error: {e}")

            self.renewal.clear()

        return result

    
    def start_up(self):
        self.run_nav()
        self.run_slam()
        self.run_http()

    def renew(self):
        result = {
            "success": True,
            "errors": []
        }
        if self.renewal:
            for p in self.renewal:
                try:
                    pgid = os.getpgid(p.pid)
                    os.killpg(pgid, signal.SIGTERM)
                    p.wait(timeout=3)
                except Exception as e:
                    result["success"] = False
                    result["errors"].append(f"renewal PID {p.pid} error: {e}")
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                        result["success"] = True
                    except Exception as e:
                        result["errors"].append(f"force kill error: {e}")
            self.renewal.clear()

        if os.path.exists(self.pbstream) and os.path.exists(self.map_yaml) and os.path.exists(self.map):
            ret = subprocess.Popen(['ros2', 'launch', 'cartographer_ros', 'fd_slam.launch.py', 'config_name:=fd.lua'], preexec_fn=os.setsid)
            self.renewal.append(ret)
            time.sleep(1)
            ret = subprocess.Popen(['ros2', 'launch', 'cartographer_ros', 'fd_occ.launch.py'], preexec_fn=os.setsid)
            self.renewal.append(ret)
            ret = subprocess.Popen(['ros2', 'launch', 'web_control', 'set_initpose.launch.py'], preexec_fn=os.setsid)
            self.renewal.append(ret)
        else:
            ret = subprocess.Popen(['ros2', 'launch', 'cartographer_ros', 'fd_slam.launch.py', 'config_name:=fd.lua', 'map_path:=""'], preexec_fn=os.setsid)
            self.renewal.append(ret)
            time.sleep(1)
            ret = subprocess.Popen(['ros2', 'launch', 'cartographer_ros', 'fd_occ.launch.py'], preexec_fn=os.setsid)
            self.renewal.append(ret)

        return result

    def remask(self):
        result = {
            "success": True,
            "errors": []
        }
        if self.maskprocesses:
            for p in self.maskprocesses:
                try:
                    pgid = os.getpgid(p.pid)
                    os.killpg(pgid, signal.SIGTERM)
                    p.wait(timeout=3)
                except Exception as e:
                    result["success"] = False
                    result["errors"].append(f"maskprocesses PID {p.pid} error: {e}")
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                        result["success"] = True
                    except Exception as e:
                        result["errors"].append(f"force kill error: {e}")
            self.maskprocesses.clear()

        if os.path.exists(self.mask) and os.path.exists(self.mask_yaml):
            self.get_logger().info('mask file exists, setting...')
            ret = subprocess.Popen(['ros2', 'launch', 'nav2_bringup', 'fd_mask.launch.py'], preexec_fn=os.setsid)
            self.maskprocesses.append(ret)
        else:
            result["success"] = False
        return result

    def kill_http(self):
        result = {
            "success": True,
            "errors": []
        }

        if self.httpproc:
            for p in self.httpproc:
                try:    
                    pgid = os.getpgid(p.pid)
                    os.killpg(pgid, signal.SIGTERM)
                    p.wait(timeout=3)
                except Exception as e:
                    result["success"] = False
                    result["errors"].append(f"processes PID {p.pid} error: {e}")
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                        result["success"] = True
                    except Exception as e:
                        result['errors'].append(f"force kill error: {e}")

            self.httpproc.clear()
        return result
    

def main(args=None):
    rclpy.init(args=args)
    node = Run()

    def signal_handler(signum, frame):
        print(f"开始清理进程.......")
        try:
            node.kill_all()
            node.kill_http()
        except Exception as e:
            print(f"清理出错: {e}")
        finally:
            rclpy.shutdown()
            sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    node.start_up()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)
    # rclpy.spin(node)
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
