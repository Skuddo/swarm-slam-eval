#!/usr/bin/env python3
import rclpy
from .odometry import OdometryNode
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped

class GroundTruthNode(OdometryNode):
    def __init__(self):
        super().__init__('ground_truth_node')
        self.get_logger().info('GroundTruthNode initialized, waiting for bag reader status...')

    def node_pose_callback(self, msg: PoseWithCovarianceStamped):
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
