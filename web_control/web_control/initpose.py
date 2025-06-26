import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, TransformStamped
from cartographer_ros_msgs.srv import GetTrajectoryStates, FinishTrajectory, StartTrajectory
from rclpy.task import Future
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import json
import time
import os


class InitialPose(Node):
    def __init__(self):
        super().__init__('initial_pose_node')

        self.declare_parameter('configuration_directory', '/default/path')
        self.declare_parameter('configuration_basename', 'default.lua')
        self.declare_parameter('json_file', '/data')

        self.config_dir = self.get_parameter('configuration_directory').get_parameter_value().string_value
        self.config_file = self.get_parameter('configuration_basename').get_parameter_value().string_value
        self.json_file = self.get_parameter('json_file').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.rviz_callback, 10)
        
        self.pubpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.cli = self.create_client(GetTrajectoryStates, '/get_trajectory_states')

        self.finish_cli = self.create_client(FinishTrajectory, '/finish_trajectory')

        self.start_cli = self.create_client(StartTrajectory, '/start_trajectory')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if os.path.exists(self.json_file):
            pose_msg = PoseWithCovarianceStamped() 
            with open(self.json_file, 'r') as f:
                pose = json.load(f)
                pose_msg.pose.pose.position.x = pose.get('x', 0.0)
                pose_msg.pose.pose.position.y = pose.get('y', 0.0)
                pose_msg.pose.pose.position.z = pose.get('z', 0.0)
                pose_msg.pose.pose.orientation.x = pose.get('qx', 0.0)
                pose_msg.pose.pose.orientation.y = pose.get('qy', 0.0)
                pose_msg.pose.pose.orientation.z = pose.get('qz', 0.0)
                pose_msg.pose.pose.orientation.w = pose.get('qw', 1.0)
                pose_msg.pose.covariance = [0.0] * 36
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'map'
            self.pubpose.publish(pose_msg)
            self.get_clock().sleep_for(Duration(seconds=1))
        else:
            self.get_logger().info('No last pose file exisit, create one')               
            
        self.timer = self.create_timer(0.2, self.timer_callback)

        self.pose = None 

    def timer_callback(self):
        try:
            now = Time()
            trans : TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', now, timeout=Duration(seconds=0.5))
            pose = {
                'x': trans.transform.translation.x,
                'y': trans.transform.translation.y,
                'z': trans.transform.translation.z,
                'qx': trans.transform.rotation.x,
                'qy': trans.transform.rotation.y,
                'qz': trans.transform.rotation.z,
                'qw': trans.transform.rotation.w,
                'timestamp': time.time()
            }
            with open(self.json_file, 'w') as f:
                json.dump(pose, f)

        except Exception as e:
            self.get_logger().warn(f"pose save failed: {str(e)}")
        
    def rviz_callback(self, msg: PoseWithCovarianceStamped):
        self.pose: Pose = msg.pose.pose
        self.get_logger().info(
            f"Received initial pose:\n"
            f"Position -> x: {self.pose.position.x:.2f}, y: {self.pose.position.y:.2f}, z: {self.pose.position.z:.2f}\n"
            f"Orientation -> x: {self.pose.orientation.x:.2f}, y: {self.pose.orientation.y:.2f}, "
            f"z: {self.pose.orientation.z:.2f}, w: {self.pose.orientation.w:.2f}"
        )
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /get_trajectory_states service...')

        self.call_get_trajectory_states()

    def call_get_trajectory_states(self):
        req = GetTrajectoryStates.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.trajectory_states_response)

    def trajectory_states_response(self, future: Future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return 
        id_list = response.trajectory_states.trajectory_id
        state_list = response.trajectory_states.trajectory_state
 
        for tid, state in zip(id_list, state_list):
            if state == 0:
                self.finish_trajectory(tid)
                self.get_clock().sleep_for(Duration(seconds=1))

        self.call_start_trajectory()


    def finish_trajectory(self, trajectory_id: int):
        req = FinishTrajectory.Request()
        req.trajectory_id = trajectory_id

        future = self.finish_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f"Finished trajectory {trajectory_id}." if not f.exception()
            else f"Failed to finish trajectory: {f.exception()}"     
        ))

    def call_start_trajectory(self):
        req = StartTrajectory.Request()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.config_file
        req.use_initial_pose = True
        req.initial_pose = self.pose
        req.relative_to_trajectory_id = 0

        future = self.start_cli.call_async(req)
        future.add_done_callback(self.on_started_trajectory)

    def on_started_trajectory(self, future):
        try:
            res = future.result()
            if res.status.code == 0:
                self.get_logger().info(f"✅ New trajectory started: ID = {res.trajectory_id}")
            else:
                self.get_logger().error(f"❌ Failed to start trajectory: {res.status.message}")
        except Exception as e:
            self.get_logger().error(f"Exception when starting trajectory: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = InitialPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




