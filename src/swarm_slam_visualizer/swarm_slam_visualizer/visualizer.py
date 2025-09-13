import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from swarm_slam_eval.qos_profiles import DATA_QOS

COLORS = [
    ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),   # Red
    ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),   # Green
    ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),   # Blue
    ColorRGBA(r=1.0, g=0.6, b=0.6, a=1.0),   # Light Red
    ColorRGBA(r=0.6, g=1.0, b=0.6, a=1.0),   # Light Green
    ColorRGBA(r=0.6, g=0.6, b=1.0, a=1.0),   # Light Blue
]

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        self.declare_parameter('num_robots', 0)
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        self.declare_parameter('nav_mode', 'imu')
        self.nav_mode = self.get_parameter('nav_mode').get_parameter_value().string_value

        self.latest_gt_poses = {}
        self.latest_nav_poses = {}
        self.gt_paths = {i: [] for i in range(self.num_robots)}
        self.nav_paths = {i: [] for i in range(self.num_robots)}

        self.nav_alignment = {i: None for i in range(self.num_robots)}

        self.marker_publishers = {}
        for i in range(self.num_robots):
            self.marker_publishers[f"gt{i}"] = self.create_publisher(
                Marker, f'/r{i}/gt_vis_marker', 10)
            self.marker_publishers[f"{self.nav_mode}{i}"] = self.create_publisher(
                Marker, f'/r{i}/{self.nav_mode}_vis_marker', 10)

        for i in range(self.num_robots):
            rid = i
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/r{rid}/gt_vis_pose',
                lambda msg, rid=rid: self.publishPose(msg, rid, gt=True),
                DATA_QOS
            )
            if self.nav_mode:
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    f'/r{rid}/{self.nav_mode}_vis_pose',
                    lambda msg, rid=rid: self.publishPose(msg, rid, gt=False),
                    DATA_QOS
                )

        self.path_publish_timer = self.create_timer(1.0, self.publishPaths)
        self.get_logger().info(f"VisualizerNode started for {self.num_robots} robots (nav_mode={self.nav_mode}).")

    def _quat_to_yaw(self, q):
        try:
            x, y, z, w = q.x, q.y, q.z, q.w
            return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
        except Exception:
            return 0.0

    def _rotate_2d(self, x, y, yaw):
        c = math.cos(yaw)
        s = math.sin(yaw)
        return c*x - s*y, s*x + c*y

    def publishPose(self, msg: PoseWithCovarianceStamped, rid: int, gt: bool = True):
        if gt:
            self.latest_gt_poses[rid] = msg
        else:
            self.latest_nav_poses[rid] = msg

        if self.nav_alignment[rid] is None and rid in self.latest_gt_poses and rid in self.latest_nav_poses:
            try:
                gt_msg = self.latest_gt_poses[rid]
                nav_msg = self.latest_nav_poses[rid]
                gt_p = gt_msg.pose.pose.position
                nav_p = nav_msg.pose.pose.position

                gt_yaw = self._quat_to_yaw(gt_msg.pose.pose.orientation)
                nav_yaw = self._quat_to_yaw(nav_msg.pose.pose.orientation)
                yaw_offset = gt_yaw - nav_yaw

                nav_x_rot, nav_y_rot = self._rotate_2d(nav_p.x, nav_p.y, yaw_offset)
                trans_x = gt_p.x - nav_x_rot
                trans_y = gt_p.y - nav_y_rot

                self.nav_alignment[rid] = (yaw_offset, trans_x, trans_y)
                self.get_logger().info(
                    f"Alignment for r{rid}: yaw={yaw_offset:.3f}, trans=({trans_x:.3f},{trans_y:.3f})"
                )
            except Exception as e:
                self.get_logger().warn(f"Failed to compute alignment for r{rid}: {e}")
                self.nav_alignment[rid] = (0.0, 0.0, 0.0)

    def publishPaths(self):
        now = self.get_clock().now().to_msg()
        for rid, pose_msg in self.latest_gt_poses.items():
            new_point = pose_msg.pose.pose.position
            self.gt_paths[rid].append(new_point)

            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = now
            marker.ns = f"gt_pose_path_r{rid}"
            marker.id = rid
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.points = self.gt_paths[rid]

            marker.scale.x = 0.2
            marker.color = COLORS[rid % len(COLORS)]
            marker.lifetime = Duration(seconds=0).to_msg()

            self.marker_publishers[f"gt{rid}"].publish(marker)

        for rid, pose_msg in self.latest_nav_poses.items():
            raw_point = pose_msg.pose.pose.position
            align = self.nav_alignment.get(rid)

            if align is None:
                aligned_x, aligned_y = raw_point.x, raw_point.y
            else:
                yaw_off, tx, ty = align
                rx, ry = self._rotate_2d(raw_point.x, raw_point.y, yaw_off)
                aligned_x, aligned_y = rx + tx, ry + ty

            p = Point()
            p.x, p.y, p.z = aligned_x, aligned_y, raw_point.z
            self.nav_paths[rid].append(p)

            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = now
            marker.ns = f"{self.nav_mode}_pose_path_r{rid}"
            marker.id = rid
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.points = self.nav_paths[rid]

            marker.scale.x = 0.2
            color_offset = len(COLORS) // 2
            marker.color = COLORS[(rid + color_offset) % len(COLORS)]
            marker.lifetime = Duration(seconds=0).to_msg()

            self.marker_publishers[f"{self.nav_mode}{rid}"].publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
