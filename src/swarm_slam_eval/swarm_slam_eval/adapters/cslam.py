import json
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from cslam_common_interfaces.msg import OptimizationResult
from swarm_slam_eval.frontend.odometry import OdometryNode
from swarm_slam_eval.qos_profiles import DATA_QOS


class CslamNode(OdometryNode):
    def __init__(self):
        super().__init__('cslam_node')

        self.odom_type = 'OptimizationResult'  
        self.mode = 'cslam'                    
        self.msg_type = OptimizationResult     

        ns = self.get_namespace().strip('/')
        self.robot_id = int(''.join(filter(str.isdigit, ns)))

        self._desired_topic = f"/r{self.robot_id}/cslam/optimized_estimates"

        self.pose_topic = None
        self._cslam_sub = None
        self.odom_publisher = None

        self._poll_timer = None

        self.get_logger().info(f"CslamNode for r{self.robot_id} initialized; waiting for {self._desired_topic} to appear.")

    def topicInfoCallback(self, msg):
        try:
            topics_list = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse topics_info JSON: {e}")
            topics_list = []

        found = False
        for t in topics_list:
            if t.get('name') == self._desired_topic:
                found = True
                break

        if found:
            self._attach_to_cslam_topic(self._desired_topic)
        else:
            if self._poll_timer is None:
                self._poll_timer = self.create_timer(0.5, self._poll_for_cslam_topic)
                self.get_logger().info(f"{self._desired_topic} not present in topics_info — polling for it until available.")

    def _poll_for_cslam_topic(self):
        try:
            topics = self.get_topic_names_and_types()
        except Exception as e:
            self.get_logger().debug(f"get_topic_names_and_types() failed: {e}")
            topics = []

        for name, types in topics:
            if name == self._desired_topic:
                self.get_logger().info(f"Discovered topic {name} in ROS topic list (types: {types}). Attaching.")
                if self._poll_timer:
                    self._poll_timer.cancel()
                    self._poll_timer = None
                self._attach_to_cslam_topic(name)
                return
            
    def _attach_to_cslam_topic(self, topic_name: str):
        if self._cslam_sub is not None:
            return

        try:
            self._cslam_sub = self.create_subscription(
                OptimizationResult,
                topic_name,
                self.poseCallback,
                DATA_QOS
            )
            self.pose_topic = topic_name

            self.odom_publisher = self.create_publisher(
                PoseWithCovarianceStamped,
                f"{self.mode}_vis_pose",
                DATA_QOS
            )

            self.get_logger().info(f"Subscribed to {topic_name} and created publisher '{self.mode}_vis_pose' for r{self.robot_id}.")
        except Exception as e:
            self.get_logger().error(f"Failed to attach to C-SLAM topic {topic_name}: {e}")

    def poseCallback(self, msg: OptimizationResult):
        if not self.is_running:
            return

        latest_pose = None
        latest_keyframe_id = -1

        for estimate in msg.estimates:
            try:
                if estimate.key.robot_id == self.robot_id:
                    if estimate.key.keyframe_id > latest_keyframe_id:
                        latest_keyframe_id = estimate.key.keyframe_id
                        latest_pose = estimate.pose
            except Exception:
                continue

        if latest_pose is None:
            return

        if not self.odom_publisher:
            self.get_logger().warn("odom_publisher not ready - cannot publish cslam_vis_pose yet.")
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.pose = latest_pose
        self.odom_publisher.publish(pose_msg)

        self.get_logger().debug(f"Published C-SLAM pose for r{self.robot_id} (kf={latest_keyframe_id})")


def main(args=None):
    rclpy.init(args=args)
    node = CslamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
