import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class SyncNode(Node):
    """
    Manages the startup and shutdown of the simulation.
    1. Waits for all robot odometry nodes to register as ready.
    2. Publishes a /start_simulation signal.
    3. Waits for all robot bag readers to report they are finished.
    4. Initiates a system shutdown.
    """
    def __init__(self):
        super().__init__('sync_node')
        self.declare_parameter('num_robots', 0)
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
 
        if self.num_robots == 0:
            self.get_logger().error("num_robots is 0, shutting down immediately.")
            rclpy.shutdown()
            return
        else:
            self.get_logger().info(f'SyncNode initialized. Waiting for {self.num_robots} robots to be ready.')
               
        self.ready_robots_count = 0
        self.finished_robots = set()

        # Publisher to signal all nodes to start
        self.start_publisher = self.create_publisher(
            Empty,
            '/start_simulation',
            10
        )

        # Service for odometry nodes to report they are ready
        self.ready_service = self.create_service(
            Trigger,
            '/register_ready',
            self.register_callback
        )

        status_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        for i in range(1, self.num_robots + 1):
            robot_id = i
            self.create_subscription(
                String,
                f'/r{robot_id}/status',
                lambda msg, rid=robot_id: self.status_callback(msg, rid),
                status_qos
            )

    def register_callback(self, request, response):
        if self.ready_robots_count < self.num_robots:
            self.ready_robots_count += 1
            self.get_logger().info(f'A robot reported ready. Total ready: {self.ready_robots_count}/{self.num_robots}')

        if self.ready_robots_count >= self.num_robots:
            self.get_logger().info('All robots are ready! Publishing start signal.')
            self.start_publisher.publish(Empty())
        
        response.success = True
        return response

    def status_callback(self, msg: String, robot_id: int):
        if msg.data == 'finished':
            if robot_id not in self.finished_robots:
                self.finished_robots.add(robot_id)
                self.get_logger().info(f"Robot {robot_id} bag has finished. Total finished: {len(self.finished_robots)}/{self.num_robots}")

            # If all robots have finished, shut down the system
            if len(self.finished_robots) >= self.num_robots:
                self.get_logger().info("All bags have finished playing. Shutting down all nodes.")
                # This will cause rclpy.spin() to exit and the program to terminate.
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()