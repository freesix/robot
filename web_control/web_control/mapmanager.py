import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from web_control_msgs.action import MapSave
from cartographer_ros_msgs.srv import WriteState
import subprocess


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        self.feedback = MapSave.Feedback()
        self.feedback.status = 0
        self._action_server = ActionServer(self, MapSave, 'save_map', self.mapCallback)
        self._write_state_cli = self.create_client(WriteState, 'write_state')        
        while not self._write_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Wait write_state start up...')
        self.get_logger().info('write_state was ready')

    async def mapCallback(self, goal_handle):
        self.feedback.status = 1
        goal_handle.publish_feedback(self.feedback)
        pbstream_path = goal_handle.request.map_path
        map_name = goal_handle.request.map_name
        resolution = goal_handle.request.resolution

        req = WriteState.Request()
        req.filename = pbstream_path
        req.include_unfinished_submaps = True

        future = self._write_state_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if not result or result.status.code != 0:
            self.feedback.status = -1
            goal_handle.publish_feedback(self.feedback)
            goal_handle.abort()
            self.get_logger().error('Save pbstream failed.')
            return MapSave.Result(ret=False)
        
        self.get_logger().info('Save pbstream success.')

        self.feedback.status = 2
        goal_handle.publish_feedback(self.feedback)

        cmd = [
            'ros2', 'run', 'cartographer_ros', 'cartographer_pbstream_to_ros_map',
                f'-pbstream_filename={pbstream_path}',
                f'-map_filestem={map_name}',
                f'-resolution={resolution}' 
        ]

        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            self.feedback.status = -1
            goal_handle.publish_feedback(self.feedback)
            self.get_logger().error(f'map conver failed.')
            goal_handle.abort()
            return MapSave.Result(ret=False)
        
        self.feedback.status = 3
        goal_handle.publish_feedback(self.feedback)

        self.get_logger().info('map save success.')
        goal_handle.succeed()
        return MapSave.Result(ret=True)
    
def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()          

