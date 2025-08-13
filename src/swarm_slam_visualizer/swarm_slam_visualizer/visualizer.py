
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped 
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        self.data_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.declare_parameter('num_robots', 0)
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        self.declare_parameter('nav_mode', 'imu')
        self.nav_mode = self.get_parameter('nav_mode').get_parameter_value().string_value
        
        self.latest_gt_poses = {} 
        self.latest_nav_poses = {} 
        self.gt_paths = {i: [] for i in range(1, self.num_robots + 1)}
        self.nav_paths = {i: [] for i in range(1, self.num_robots + 1)}
        self.marker_publishers = {}
        for i in range(1, self.num_robots + 1):
            self.marker_publishers[f"gt{i}"] = self.create_publisher(
                Marker, f'/r{i}/gt_vis_marker', 10)
            self.marker_publishers[f"{self.nav_mode}{i}"] = self.create_publisher(
                Marker, f'/r{i}/{self.nav_mode}_vis_marker', 10)

        self.path_colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),   # Red
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),   # Green
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),   # Blue
            ColorRGBA(r=1.0, g=0.6, b=0.6, a=1.0),   # Light Red (pinkish tint)
            ColorRGBA(r=0.6, g=1.0, b=0.6, a=1.0),   # Light Green (minty tint)
            ColorRGBA(r=0.6, g=0.6, b=1.0, a=1.0),   # Light Blue (sky blue)
        ]
        
        self.data_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # ground truth pose
        for i in range(1, self.num_robots + 1):
            robot_id = i
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/r{robot_id}/gt_vis_pose',
                lambda msg, rid=robot_id: self.pose_callback(msg, rid),
                self.data_qos
            )

            if self.nav_mode != '':
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    f'/r{robot_id}/{self.nav_mode}_vis_pose',
                    lambda msg, rid=robot_id: self.pose_callback(msg, rid, False),
                    self.data_qos
                )


        self.path_publish_timer = self.create_timer(1.0, self.publish_paths_callback)
        self.get_logger().info(f"VisualizerNode started for {self.num_robots} robots.")

    def pose_callback(self, msg: PoseWithCovarianceStamped, robot_id: int, gt: bool=True):
        if gt:
            self.latest_gt_poses[robot_id] = msg
        else:
            self.latest_nav_poses[robot_id] = msg

    def publish_paths_callback(self):
        for robot_id, pose_msg in self.latest_gt_poses.items():
            publisher = self.marker_publishers[f"gt{robot_id}"]
            new_point = pose_msg.pose.pose.position
            self.gt_paths[robot_id].append(new_point)
            
            path_marker = Marker()
            path_marker.header.frame_id = "world"
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = "gps_ground_truth_path"
            path_marker.id = robot_id 
            path_marker.type = Marker.LINE_STRIP 
            path_marker.action = Marker.ADD
            path_marker.points = self.gt_paths[robot_id]
            
            path_marker.scale.x = 0.3
            path_marker.color = self.path_colors[(robot_id - 1) % len(self.path_colors)] # Robust color indexing
            path_marker.lifetime = Duration(seconds=0).to_msg()
            publisher.publish(path_marker)

        for robot_id, pose_msg in self.latest_nav_poses.items():
            publisher = self.marker_publishers[f"{self.nav_mode}{robot_id}"]
            new_point = pose_msg.pose.pose.position
            self.nav_paths[robot_id].append(new_point)
            
            path_marker = Marker()
            path_marker.header.frame_id = "world"
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = "imu_pose_path"
            path_marker.id = robot_id 
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.points = self.nav_paths[robot_id]
            
            path_marker.scale.x = 0.2
            color_offset = len(self.path_colors) // 2 
            path_marker.color = self.path_colors[(robot_id - 1 + color_offset) % len(self.path_colors)]
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