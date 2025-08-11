import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.is_running = False
        self.is_registered = False
        self.available_topics = []
        self.ground_truth_topic = None
        self.odom_publisher = None

        # QoS profile for latched topics
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.get_logger().info('Initializing. Waiting for local bag reader to be ready...')
        self.create_subscription(
            String, 
            'status', 
            self.on_bag_ready_callback, 
            qos_profile
        )

    def on_bag_ready_callback(self, msg):
        if msg.data == 'ready' and not self.is_registered:
            self.get_logger().info('Local bag reader is ready.')
            qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                String, 
                'topics_info', 
                self.on_topics_info_callback, 
                qos_profile
            )

    def on_topics_info_callback(self, msg):
        if self.is_registered:
            return

        self.is_registered = True
        self.available_topics = json.loads(msg.data)
        
        # Find ground truth topic
        for topic_info in self.available_topics:
            if 'ground_truth' in topic_info['name'] and 'Odometry' in topic_info['type']:
                self.ground_truth_topic = topic_info['name']
                break

        if self.ground_truth_topic:
            self.get_logger().info(f"Found ground truth topic: {self.ground_truth_topic}")
            self.create_subscription(
                Odometry,
                self.ground_truth_topic,
                self.ground_truth_callback,
                10
            )
            
            qos_profile = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
            )

            # Create publisher for visualization
            self.odom_publisher = self.create_publisher(
                PoseWithCovarianceStamped,
                'visualization_pose',
                qos_profile=qos_profile
            )

        # Register with sync node
        self.register_with_sync_node()

    def register_with_sync_node(self):
        self.reg_client = self.create_client(Trigger, '/register_ready')
        while not self.reg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sync service not available, waiting...')
        
        self.reg_client.call_async(Trigger.Request())
        self.get_logger().info('Registered with sync node. Waiting for start signal.')
        self.create_subscription(Empty, '/start_simulation', self.on_start_callback, 10)

    def ground_truth_callback(self, msg):
        if not self.is_running:
            return
            
        # Convert Odometry to PoseWithCovarianceStamped for visualization
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose
        
        if self.odom_publisher:
            self.odom_publisher.publish(pose_msg)

    def on_start_callback(self, msg):
        if not self.is_running:
            self.is_running = True
            self.get_logger().info('Starting odometry processing')

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()