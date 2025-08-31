import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from swarm_slam_eval.qos_profiles import DATA_QOS

class CslamNode(Node):
    def __init__(self):
        super().__init__('cslam_node')
        
        # C-SLAM's odometry frontend (RTAB-Map) publishes on 'odom' within the robot's namespace
        self.create_subscription(
            Odometry,
            'odom',  # Subscribes to /rX/odom
            self.odom_callback,
            DATA_QOS
        )
        
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'cslam_vis_pose', # Publishes to /rX/cslam_vis_pose
            DATA_QOS
        )
        
        self.get_logger().info('C-SLAM adapter node started. Waiting for odometry data...')

    def odom_callback(self, msg: Odometry):
        # Create a PoseWithCovarianceStamped message and populate it from the Odometry message
        pose_stamped_msg = PoseWithCovarianceStamped()
        pose_stamped_msg.header = msg.header
        pose_stamped_msg.pose = msg.pose
        
        self.pose_publisher.publish(pose_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CslamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()