import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from .qos_profiles import SIGNAL_QOS

class SyncNode(Node):
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

        self.start_publisher = self.create_publisher(
            Empty,
            '/start_simulation',
            SIGNAL_QOS
        )

        self.ready_service = self.create_service(
            Trigger,
            '/register_ready',
            self.readyCallback
        )
        
        for i in range(self.num_robots):
            robot_id = i
            self.create_subscription(
                String,
                f'/r{robot_id}/status',
                lambda msg, rid=robot_id: self.statusCallback(msg, rid),
                SIGNAL_QOS
            )

    def readyCallback(self, _, response):
        if self.ready_robots_count < self.num_robots:
            self.ready_robots_count += 1
            self.get_logger().info(f'A robot reported ready. Total ready: {self.ready_robots_count}/{self.num_robots}')
        
        if self.ready_robots_count >= self.num_robots:
            self.get_logger().info('All robots are ready! Publishing start signal (burst).')
            def publish_burst():
                wait_timeout = 5.0
                poll_interval = 0.05
                waited = 0.0

                while waited < wait_timeout:
                    subs = self.start_publisher.get_subscription_count()
                    if subs >= self.num_robots:
                        break
                    time.sleep(poll_interval)
                    waited += poll_interval

                if waited >= wait_timeout:
                    self.get_logger().warn(
                        f"Timeout waiting for subscribers to /start_simulation (have {self.start_publisher.get_subscription_count()}, "
                        f"expected {self.num_robots}); publishing anyway."
                    )

                for _ in range(8):
                    try:
                        self.start_publisher.publish(Empty())
                    except Exception as e:
                        self.get_logger().warn(f"Failed to publish start: {e}")
                    time.sleep(0.12)


            threading.Thread(target=publish_burst, daemon=True).start()

        response.success = True
        return response

    def statusCallback(self, msg: String, robot_id: int):
        if msg.data == 'finished':
            if robot_id not in self.finished_robots:
                self.finished_robots.add(robot_id)
                self.get_logger().info(f"Robot {robot_id} bag has finished. Total finished: {len(self.finished_robots)}/{self.num_robots}")

            if len(self.finished_robots) >= self.num_robots:
                self.get_logger().info("All bags have finished playing. Shutting down all nodes.")
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()