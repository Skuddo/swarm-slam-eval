#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import rclpy.time

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.is_running = False
        self.is_registered = False
        self.available_topics = []
        self.ground_truth_topic = None
        self.odom_publisher = None

        # subscribe to bag reader status (latched / transient-local)
        status_qos = QoSProfile(depth=1)
        status_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        status_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(String, 'status', self.on_bag_ready_callback, status_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('OdometryNode initialized, waiting for bag reader status...')

    def on_bag_ready_callback(self, msg: String):
        if msg.data == 'ready' and not self.is_registered:
            self.get_logger().info('Local bag reader is ready — subscribing to topics_info')
            # subscribe to topics_info (transient-local so we get the last published list)
            topics_qos = QoSProfile(depth=1)
            topics_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            topics_qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.create_subscription(String, 'topics_info', self.on_topics_info_callback, topics_qos)

    def on_topics_info_callback(self, msg: String):
        if self.is_registered:
            return

        try:
            self.available_topics = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse topics_info JSON: {e}")
            return

        # prefer matching by message type (nav_msgs/msg/Odometry)
        odom_type_strings = ('nav_msgs/msg/Odometry', 'nav_msgs/Odometry', 'nav_msgs/msg/Odometry')
        found = False
        for topic_info in self.available_topics:
            name = topic_info.get('name', '')
            typ = topic_info.get('type', '')
            if typ in odom_type_strings or 'Odometry' in typ:
                # choose first matching odometry topic
                self.ground_truth_topic = name
                found = True
                break

        if not found:
            self.get_logger().warn('No odometry topic found in topics_info. Received: ' + str(self.available_topics))
            self.is_registered = True  # mark as registered to avoid reprocessing bad data
            return

        self.is_registered = True
        self.get_logger().info(f"Found ground truth topic: {self.ground_truth_topic}")

        # Subscribe to ground-truth odometry (use a reasonable QoS)
        odom_qos = QoSProfile(depth=10)
        odom_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        odom_qos.durability = QoSDurabilityPolicy.VOLATILE
        self.create_subscription(Odometry, self.ground_truth_topic, self.ground_truth_callback, odom_qos)

        # publisher for visualization pose (PoseWithCovarianceStamped)
        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        pub_qos.durability = QoSDurabilityPolicy.VOLATILE
        self.odom_publisher = self.create_publisher(PoseWithCovarianceStamped, 'visualization_pose', pub_qos)

        # Register with sync node (non-blocking)
        self.register_with_sync_node()

    def register_with_sync_node(self):
        # create client and call register service
        self.reg_client = self.create_client(Trigger, '/register_ready')
        if not self.reg_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Sync service '/register_ready' not available (timeout). Continuing without registration.")
            # still subscribe to start topic so we can operate
            self.create_subscription(Empty, '/start_simulation', self.on_start_callback, 10)
            return

        # call once (don't block waiting on response)
        future = self.reg_client.call_async(Trigger.Request())
        # optionally add a done callback
        def _on_reg_done(fut):
            try:
                res = fut.result()
                self.get_logger().info("Registered with sync node.")
            except Exception as e:
                self.get_logger().warn(f"Register service call failed: {e}")
        future.add_done_callback(_on_reg_done)

        # subscribe to start signal (normal QoS)
        self.create_subscription(Empty, '/start_simulation', self.on_start_callback, 10)
        self.get_logger().info('Waiting for /start_simulation...')

    def ground_truth_callback(self, msg):
        if not self.is_running:
            return

        # Publish PoseWithCovarianceStamped as before
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose

        if self.odom_publisher:
            self.odom_publisher.publish(pose_msg)

        # Also broadcast TF transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "world"  # or "map" or whichever fixed frame you use
        t.child_frame_id = "base_enu"  # this frame must match your RViz Fixed Frame or TF tree

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def on_start_callback(self, msg: Empty):
        if not self.is_running:
            self.is_running = True
            self.get_logger().info('Starting odometry processing')

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
