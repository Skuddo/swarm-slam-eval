import rclpy
import numpy as np
from .odometry import OdometryNode
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation

GRAVITY = 9.80665 

class ImuNode(OdometryNode):
    def __init__(self):
        # Call parent to set up QoS, start signal sub, etc.
        super().__init__('imu_node')
        self.odom_type = 'Imu'
        self.mode = 'imu'
        self.msg_type = Imu
        # --- Child-Specific State for IMU Integration ---
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = None
        self.last_timestamp = None

        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'gt_vis_pose',
            self._initial_pose_callback,
            self.data_qos
        )

        self.get_logger().info('ImuNode waiting for initial pose from gt_vis_pose...')

    def on_topics_info_callback(self, msg: String):
        super().on_topics_info_callback(msg)

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        if not self.is_initialized:
            pos = msg.pose.pose.position
            self.position = np.array([pos.x, pos.y, pos.z])
            self.orientation = msg.pose.pose.orientation
            
            self.last_timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.is_initialized = True

            o = self.orientation
            self.get_logger().info(f"IMU odometry initialized to start pose: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
            self.get_logger().info(f"and orientation: ({o.x:.2f}, {o.y:.2f}, {o.z:.2f})")
            
            # Destroy the subscription so this callback doesn't run again
            self.destroy_subscription(self.initial_pose_sub)
            self.register_self()

    def pose_callback(self, msg: Imu):
        if not self.is_initialized or not self.is_running:
            return

        current_timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.last_timestamp is None:
            self.last_timestamp = current_timestamp
            return
            
        dt = (current_timestamp - self.last_timestamp).nanoseconds / 1e9
        self.last_timestamp = current_timestamp
        if dt <= 0 or dt > 0.5: return
        
        self.orientation = msg.orientation
        q = self.orientation
        current_rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])

        # 2. Get the body-frame acceleration and remove gravity
        body_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z - GRAVITY
        ])

        world_accel = current_rotation.apply(body_accel)

        self.velocity += world_accel * dt
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