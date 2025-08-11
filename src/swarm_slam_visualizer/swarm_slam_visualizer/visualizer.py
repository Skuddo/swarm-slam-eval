import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
import os

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        
        # QoS for latched topics
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # Subscribe to topics info to auto-configure
        self.create_subscription(
            String,
            'topics_info',
            self.on_topics_info_callback,
            qos_profile
        )
        
        self.get_logger().info("Visualizer node ready, waiting for topics info...")

    def on_topics_info_callback(self, msg):
        self.get_logger().info("Received topics info, visualizer is configured")

def generate_launch_description():
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare
    import os

    config_dir = FindPackageShare('swarm_slam_visualizer').find('swarm_slam_visualizer')
    rviz_config_path = os.path.join(config_dir, 'config', 'ground_truth_visualization.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_path,
            description='Path to RViz config file'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        Node(
            package='swarm_slam_visualizer',
            executable='visualizer_node',
            name='visualizer_node',
            output='screen'
        )
    ])

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()