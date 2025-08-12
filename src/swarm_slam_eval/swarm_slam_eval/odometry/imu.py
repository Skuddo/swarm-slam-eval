#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String

class ImuNode(Node):
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

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
