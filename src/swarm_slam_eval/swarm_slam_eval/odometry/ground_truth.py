#!/usr/bin/env python3
import json
import rclpy
from .odometry import OdometryNode
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped

class GroundTruthNode(OdometryNode):
    def __init__(self):
        super().__init__('ground_truth_node')
        self.create_subscription(
            String, 'status',
            self.on_bag_ready_callback,
            self.signal_qos
            )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('GroundTruthNode initialized, waiting for bag reader status...')

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
                self.ground_truth_topic = name
                found = True
                self.get_logger().info(f"Found ground truth topic: {self.ground_truth_topic}")
                break

        if not found:
            self.get_logger().warn('No odometry topic found in topics_info. Received: ' + str(self.available_topics))
            return

        self.is_registered = True

        self.create_subscription(Odometry, self.ground_truth_topic, self.ground_truth_callback, self.data_qos)

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

    def ground_truth_callback(self, msg):
        if not self.is_running:
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose

        if self.odom_publisher:
            self.odom_publisher.publish(pose_msg)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "world"
        robot_namespace = self.get_namespace().strip('/')
        t.child_frame_id = f"{robot_namespace}/base_enu"
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def on_start_callback(self, msg: Empty):
        if not self.is_running:
            self.is_running = True
            self.get_logger().info('Starting ground truth processing')

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
