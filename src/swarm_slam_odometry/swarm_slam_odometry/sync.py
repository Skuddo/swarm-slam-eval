import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

class SyncNode(Node):
    """
    This node ensures that all robot nodes are ready before starting the simulation.
    It provides a service for odometry nodes to register their readiness.
    Once all nodes are ready, it publishes a message to a global start topic.
    """
    def __init__(self):
        super().__init__('sync_node')
        self.declare_parameter('num_robots', 0)
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        self.ready_robots = 0

        self.get_logger().info(f'SyncNode initialized. Waiting for {self.num_robots} robots to be ready.')

        # Publisher to signal all nodes to start
        self.start_publisher = self.create_publisher(Empty, '/start_simulation', 10)

        # Service for odometry nodes to report they are ready
        self.ready_service = self.create_service(
            Trigger,
            '/register_ready',
            self.register_callback
        )

    def register_callback(self, request, response):
        self.ready_robots += 1
        self.get_logger().info(f'A robot has reported ready. Total ready: {self.ready_robots}/{self.num_robots}')

        if self.ready_robots >= self.num_robots:
            self.get_logger().info('All robots are ready! Publishing start signal.')
            self.start_publisher.publish(Empty())
        
        response.success = True
        response.message = 'Successfully registered as ready.'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()