import rclpy
import numpy as np
from .odometry import OdometryNode
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped

class ImuNode(OdometryNode):
    def __init__(self):
        # Call parent to set up QoS, start signal sub, etc.
        super().__init__('imu_node')
        self.odom_type = 'Imu'
        self.mode = 'imu'
        # --- Child-Specific State for IMU Integration ---
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = None
        self.last_timestamp = None

        self.get_logger().info('ImuNode initialized, waiting for bag reader status...')

    def pose_callback(self, msg: Imu):
        if not self.is_running:
            return

        current_timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.last_timestamp is None:
            self.last_timestamp = current_timestamp
            self.orientation = msg.orientation
            return
        dt = (current_timestamp - self.last_timestamp).nanoseconds / 1e9
        self.last_timestamp = current_timestamp
        if dt > 0.5: return

        self.orientation = msg.orientation
        accel = msg.linear_acceleration
        self.velocity += np.array([accel.x, accel.y, accel.z]) * dt
        self.position += self.velocity * dt

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = "world"
        pose_msg.pose.pose.position.x = self.position[0]
        pose_msg.pose.pose.position.y = self.position[1]
        pose_msg.pose.pose.position.z = self.position[2]
        pose_msg.pose.pose.orientation = self.orientation

        if self.odom_publisher:
            self.odom_publisher.publish(pose_msg)

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