from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class OdometryNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.is_running = False
        self.is_registered = False
        self.available_topics = []
        self.ground_truth_topic = None
        self.odom_publisher = None

        self.signal_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1, 
        )
        self.data_qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10, 
        )
        
        self.get_logger().info(f"'{node_name}' initialized and waiting for start signal.")


    def on_start_callback(self, msg: Empty):
        if not self.is_running:
            self.is_running = True
            self.get_logger().info('Start signal received. Processing data.')

    def publish_odometry(self, pose_msg: PoseWithCovarianceStamped, child_frame_id: str):
        if not self.is_running:
            return

        # Publish the standardized pose message
        self.pose_publisher.publish(pose_msg)

        # Broadcast the TF transform
        t = TransformStamped()
        t.header = pose_msg.header
        t.header.frame_id = "world"
        t.child_frame_id = f"{self.get_namespace().strip('/')}/{child_frame_id}"
        t.transform.translation = pose_msg.pose.pose.position
        t.transform.rotation = pose_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)