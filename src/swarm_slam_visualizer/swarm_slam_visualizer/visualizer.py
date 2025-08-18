
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped 
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

DATA_QOS = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

COLORS = [
    ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),   # Red
    ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),   # Green
    ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),   # Blue
    ColorRGBA(r=1.0, g=0.6, b=0.6, a=1.0),   # Light Red (pinkish tint)
    ColorRGBA(r=0.6, g=1.0, b=0.6, a=1.0),   # Light Green (minty tint)
    ColorRGBA(r=0.6, g=0.6, b=1.0, a=1.0),   # Light Blue (sky blue)
]

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        self.declare_parameter('num_robots', 0)
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        self.declare_parameter('nav_mode', 'imu')
        self.nav_mode = self.get_parameter('nav_mode').get_parameter_value().string_value
        
        # Path data structures
        self.latest_gt_poses = {} 
        self.latest_nav_poses = {} 
        self.gt_paths = {i: [] for i in range(1, self.num_robots + 1)}
        self.nav_paths = {i: [] for i in range(1, self.num_robots + 1)}
        
        # Marker publishers dict
        self.marker_publishers = {}
        for i in range(1, self.num_robots + 1):
            self.marker_publishers[f"gt{i}"] = self.create_publisher(
                Marker, f'/r{i}/gt_vis_marker', 10)
            self.marker_publishers[f"{self.nav_mode}{i}"] = self.create_publisher(
                Marker, f'/r{i}/{self.nav_mode}_vis_marker', 10)
            
        # GT and nav pose
        for i in range(1, self.num_robots + 1):
            rid = i
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/r{rid}/gt_vis_pose',
                lambda msg, rid=rid: self.publishPose(msg, rid),
                DATA_QOS
            )

            if self.nav_mode != '':
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    f'/r{rid}/{self.nav_mode}_vis_pose',
                    lambda msg, rid=rid: self.publishPose(msg, rid, False),
                    DATA_QOS
                )


        self.path_publish_timer = self.create_timer(1.0, self.publishPaths)
        self.get_logger().info(f"VisualizerNode started for {self.num_robots} robots.")

    def publishPose(self, msg: PoseWithCovarianceStamped, rid: int, gt: bool=True):
        if gt:
            self.latest_gt_poses[rid] = msg
        else:
            self.latest_nav_poses[rid] = msg

    def publishPaths(self):
        for rid, pose_msg in self.latest_gt_poses.items():
            publisher = self.marker_publishers[f"gt{rid}"]
            new_point = pose_msg.pose.pose.position
            self.gt_paths[rid].append(new_point)
            
            path_marker = Marker()
            path_marker.header.frame_id = "world"
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = "gps_ground_truth_path"
            path_marker.id = rid 
            path_marker.type = Marker.LINE_STRIP 
            path_marker.action = Marker.ADD
            path_marker.points = self.gt_paths[rid]
            
            path_marker.scale.x = 0.3
            path_marker.color = COLORS[(rid - 1) % len(COLORS)]
            path_marker.lifetime = Duration(seconds=0).to_msg()
            publisher.publish(path_marker)

        for rid, pose_msg in self.latest_nav_poses.items():
            publisher = self.marker_publishers[f"{self.nav_mode}{rid}"]
            new_point = pose_msg.pose.pose.position
            self.nav_paths[rid].append(new_point)
            
            path_marker = Marker()
            path_marker.header.frame_id = "world"
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = "imu_pose_path"
            path_marker.id = rid 
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.points = self.nav_paths[rid]
            
            path_marker.scale.x = 0.2
            color_offset = len(COLORS) // 2 
            path_marker.color = COLORS[(rid - 1 + color_offset) % len(COLORS)]
            path_marker.lifetime = Duration(seconds=0).to_msg()
            publisher.publish(path_marker)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()