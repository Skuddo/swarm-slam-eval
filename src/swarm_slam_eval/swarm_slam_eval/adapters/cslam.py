import rclpy
from swarm_slam_eval.frontend.odometry import OdometryNode
from cslam_common_interfaces.msg import OptimizationResult
from geometry_msgs.msg import PoseWithCovarianceStamped

class CslamNode(OdometryNode):
    def __init__(self):
        # Initialize the parent OdometryNode
        super().__init__('cslam_node')
        
        # 1. Configure the parent class for C-SLAM
        self.odom_type = 'OptimizationResult'  # For finding the topic in topics_info
        self.mode = 'cslam'                    # Sets the output topic to '/rX/cslam/viz/pose'
        self.msg_type = OptimizationResult     # The message type to subscribe to
        
        # Determine this robot's ID from its namespace (e.g., '/r0' -> 0)
        ns = self.get_namespace().strip('/')
        self.robot_id = int(''.join(filter(str.isdigit, ns))) if ns else -1
        
        self.get_logger().info(f'CslamNode for r{self.robot_id} waiting for initialization...')

    # 2. Implement the poseCallback to process C-SLAM data
    def poseCallback(self, msg: OptimizationResult):
        if not self.is_running:
            return
            
        latest_pose = None
        latest_keyframe_id = -1

        # Find the single latest pose for this specific robot from all estimates
        for estimate in msg.estimates:
            if estimate.key.robot_id == self.robot_id:
                if estimate.key.keyframe_id > latest_keyframe_id:
                    latest_keyframe_id = estimate.key.keyframe_id
                    latest_pose = estimate.pose

        # If we found a valid pose, publish it for the visualizer
        if latest_pose and self.odom_publisher:
            pose_msg = PoseWithCovarianceStamped()
            
            # Stamp with the current simulation time and set the frame for RViz
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world'
            
            # The message from C-SLAM is a Pose, but the output needs PoseWithCovariance
            pose_msg.pose.pose = latest_pose
            
            # self.odom_publisher was created in the base class on the correct topic
            self.odom_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CslamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()