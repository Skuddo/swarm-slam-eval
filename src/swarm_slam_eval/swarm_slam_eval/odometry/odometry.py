import json
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class OdometryNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.is_running = False
        self.is_registered = False
        self.available_topics = []
        self.pose_topic = None
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
        self.create_subscription(
            String, 'status',
            self.on_bag_ready_callback,
            self.signal_qos
            )
        self.tf_broadcaster = TransformBroadcaster(self)

    def on_bag_ready_callback(self, msg: String):
        if msg.data == 'ready' and not self.is_registered:
            self.get_logger().info('Local bag reader is ready — subscribing to topics_info')
            self.create_subscription(
                String, 'topics_info',
                self.on_topics_info_callback,
                self.signal_qos
                )

    def on_topics_info_callback(self, msg: String):
        if self.is_registered:
            return

        try:
            self.available_topics = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse topics_info JSON: {e}")
            return

        odom_type_strings = ('nav_msgs/msg/Odometry', 'nav_msgs/Odometry', 'nav_msgs/msg/Odometry')
        found = False
        for topic_info in self.available_topics:
            name = topic_info.get('name', '')
            typ = topic_info.get('type', '')
            if typ in odom_type_strings or 'Odometry' in typ:
                self.pose_topic = name
                found = True
                self.get_logger().info(f"Found ground truth topic: {self.pose_topic}")
                break

        if not found:
            self.get_logger().warn('No odometry topic found in topics_info. Received: ' + str(self.available_topics))
            return

        self.is_registered = True

        self.create_subscription(
            Odometry,
            self.pose_topic,
            self.node_pose_callback,
            self.data_qos
            )

        self.odom_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'gt_viz_pose', self.data_qos
            )

        self.register_with_sync_node()

    def register_with_sync_node(self):
        self.reg_client = self.create_client(Trigger, '/register_ready')
        if not self.reg_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Sync service '/register_ready' not available (timeout). Continuing without registration.")
            self.create_subscription(
                Empty, '/start_simulation',
                self.on_start_callback, 
                self.signal_qos
                )
            return

        future = self.reg_client.call_async(Trigger.Request())
        def _on_reg_done(fut):
            try:
                res = fut.result()
                self.get_logger().info("Registered with sync node.")
            except Exception as e:
                self.get_logger().warn(f"Register service call failed: {e}")
        future.add_done_callback(_on_reg_done)

        self.create_subscription(
            Empty, '/start_simulation',
            self.on_start_callback,
            self.signal_qos
            )
        self.get_logger().info('Waiting for /start_simulation...')

            
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