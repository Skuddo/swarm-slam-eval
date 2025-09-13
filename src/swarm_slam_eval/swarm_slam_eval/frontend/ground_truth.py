import rclpy
from .odometry import OdometryNode
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

class GroundTruthNode(OdometryNode):
    def __init__(self):
        super().__init__('ground_truth_node')
        self.odom_type = 'Odometry'
        self.mode = 'gt'
        self.msg_type = Odometry
        self.declare_parameter('offset', False)
        self.offset = self.get_parameter('offset').get_parameter_value().bool_value

        self.get_logger().info('GroundTruthNode initialized, waiting for bag reader status...')
        if self.offset:
            self.declare_parameter('initial_x', 0.0)
            self.declare_parameter('initial_y', 0.0)
            self.declare_parameter('offset_x', 0.0)
            self.declare_parameter('offset_y', 0.0)
            init_x = self.get_parameter('initial_x').get_parameter_value().double_value
            init_y = self.get_parameter('initial_y').get_parameter_value().double_value
            offset_x = self.get_parameter('offset_x').get_parameter_value().double_value
            offset_y = self.get_parameter('offset_y').get_parameter_value().double_value

            self.initial_coords = (init_x - offset_x, init_y - offset_y)
            self.get_logger().info(f'GroundTruthNode offset x: {self.initial_coords[0]} y: {self.initial_coords[1]}')

    def poseCallback(self, msg: PoseWithCovarianceStamped):
        if not self.is_running:
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose

        if self.offset:
            pose_msg.pose.pose.position.x -= self.initial_coords[0]
            pose_msg.pose.pose.position.y -= self.initial_coords[1]
            for i in range(2):
                for j in range(6):
                    pose_msg.pose.covariance[i * 6 + j] = max(
                        pose_msg.pose.covariance[i * 6 + j], 1e-6
                    )

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
