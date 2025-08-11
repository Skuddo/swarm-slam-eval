# Final, corrected visualizer.py

import rclpy
from rclpy.node import Node
# FIX: Import the correct message type
from geometry_msgs.msg import PoseWithCovarianceStamped, Point 
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        self.declare_parameter('num_robots', 3) # Set a default for standalone testing
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        
        # ... colors, data storage, publishers are the same ...
        self.latest_gt_poses = {} 
        self.ground_truth_paths = {i: [] for i in range(1, self.num_robots + 1)}
        self.marker_publishers = {
            i: self.create_publisher(Marker, f'/r{i}/visualization_marker', 10)
            for i in range(1, self.num_robots + 1)
        }

        self.path_colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),  # Red
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),  # Green 
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),  # Blue 
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8),  # Yellow
            ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8),  # Magenta
            ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8),  # Cyan
        ]

        best_effort_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        for i in range(1, self.num_robots + 1):
            robot_id = i
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/r{robot_id}/visualization_pose',
                lambda msg, rid=robot_id: self.ground_truth_callback(msg, rid),
                best_effort_qos
            )

        self.path_publish_timer = self.create_timer(1.0, self.publish_paths_callback)
        self.get_logger().info(f"VisualizerNode started for {self.num_robots} robots.")

    def ground_truth_callback(self, msg: PoseWithCovarianceStamped, robot_id: int):
        self.latest_gt_poses[robot_id] = msg

    def publish_paths_callback(self):
        for robot_id, pose_msg in self.latest_gt_poses.items():
            publisher = self.marker_publishers[robot_id]
            new_point = pose_msg.pose.pose.position
            self.ground_truth_paths[robot_id].append(new_point)
            
            path_marker = Marker()
            path_marker.header.frame_id = "world"
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = "gps_ground_truth_path"
            path_marker.id = 1
            path_marker.type = Marker.SPHERE_LIST
            path_marker.action = Marker.ADD
            path_marker.points = self.ground_truth_paths[robot_id]
            
            path_marker.scale.x = 0.3
            path_marker.scale.y = 0.3
            path_marker.scale.z = 0.3
            
            path_marker.color = self.path_colors[robot_id - 1]
            path_marker.lifetime = Duration(seconds=0).to_msg() # Persist forever
            publisher.publish(path_marker)

# ... main function is the same ...
def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()