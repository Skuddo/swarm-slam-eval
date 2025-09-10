import json
import threading
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseWithCovarianceStamped
from swarm_slam_eval.qos_profiles import SIGNAL_QOS, DATA_QOS

class OdometryNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.is_running = False
        self.is_registered = False
        self.is_initialized = False
        self.available_topics = []
        self.pose_topic = None
        self.odom_publisher = None
        self.odom_type = None
        self.mode = None
        self.msg_type = None

        self.create_subscription(
            String, 'status',
            self.bagReadyCallback,
            SIGNAL_QOS
            )
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.reg_client = self.create_client(Trigger, '/register_ready')

    # Register the node on thread
    def register(self):
        thread = threading.Thread(target=self.registerWorker, daemon=True)
        thread.start()

    # Register the node as worker
    def registerWorker(self):
        while not self.reg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Sync service not available, waiting...")
        
        future = self.reg_client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self.get_logger().info("Successfully registered with sync node."))

    # Bag ready callback, starts the subscription to get topic info
    def bagReadyCallback(self, msg: String):
        if msg.data == 'ready' and not self.is_registered:
            self.get_logger().info('Local bag reader is ready — subscribing to topics_info')
            self.create_subscription(
                String, 'topics_info',
                self.topicInfoCallback,
                SIGNAL_QOS
                )
            
    # Topic info callback, start the ready signal subscription
    def topicInfoCallback(self, msg: String):
        if self.is_registered:
            return

        try:
            self.available_topics = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse topics_info JSON: {e}")
            return

        odom_type_strings = (f'nav_msgs/msg/{self.odom_type}', f'nav_msgs/{self.odom_type}')
        found = False
        for topic_info in self.available_topics:
            name = topic_info.get('name', '')
            type = topic_info.get('type', '')
            if type in odom_type_strings or self.odom_type in type:
                self.pose_topic = name
                found = True
                self.get_logger().info(f"Found {self.mode} topic: {self.pose_topic}")
                break

        if not found:
            self.get_logger().warn(f'No {self.odom_type} topic found in topics_info. Received: ' + str(self.available_topics))
            return

        self.create_subscription(
            self.msg_type,
            self.pose_topic,
            self.poseCallback,
            DATA_QOS
            )
        
        self.odom_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            f'{self.mode}_vis_pose',
            DATA_QOS
            )
        
        self.create_subscription(
            Empty, 
            '/start_simulation',
            self.startCallback,
            SIGNAL_QOS
            )

        if self.mode == 'gt':
            self.register()
            self.is_registered = True

        self.get_logger().info('Ready. Waiting for /start_simulation signal...')            
    
    # On start callback, starts the node
    def startCallback(self, _):
        if not self.is_running:
            self.is_running = True
            self.get_logger().info('Start signal received. Processing data.')
