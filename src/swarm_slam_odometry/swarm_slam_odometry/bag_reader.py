import importlib
import json
import yaml
import os
import time
import pyproj
import threading
import rclpy
import rclpy.serialization
from rclpy.node import Node
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from std_msgs.msg import String, Empty
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix
from collections import deque

class BagReaderNode(Node):
    def __init__(self):
        super().__init__('bag_reader_node')

        # Parameters
        self.declare_parameter('bag_path', '')
        self.declare_parameter('buffer_size', 200)
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('sim_rate', 1.0)

        bag_path = self.get_parameter('bag_path').get_parameter_value().string_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value
        self.sim_rate = self.get_parameter('sim_rate').get_parameter_value().double_value
        origin_lat = self.get_parameter('origin_lat').get_parameter_value().double_value
        origin_lon = self.get_parameter('origin_lon').get_parameter_value().double_value

        if not bag_path:
            self.get_logger().error("'bag_path' parameter is not set. Shutting down.")
            rclpy.shutdown()
            return

        config_path = os.path.join(os.path.dirname(bag_path), 'config.yaml')
        if not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found at {config_path}")
            rclpy.shutdown()
            return

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        ground_cfg = self.config.get('ground', {})
        topic_mappings = ground_cfg.get('topics', {})
        self.get_logger().info(f"Loaded topic config: {list(topic_mappings.keys())}")

        self.storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        self.converter_options = ConverterOptions('', '')
        self.reader = SequentialReader()

        try:
            self.reader.open(self.storage_options, self.converter_options)
        except Exception as e:
            self.get_logger().error(f"Failed to open bag file: {e}")
            rclpy.shutdown()
            return

        all_topics_and_types = {t.name: t.type for t in self.reader.get_all_topics_and_types()}

        self.topic_publishers = {}
        self.msg_types = {}
        self.topic_hz = {}
        self.last_pub_time = {}
        self.src_to_dst = {}

        for dst_name, cfg in topic_mappings.items():
            src_topic = cfg['src']
            hz = cfg['hz']
            actual_type = all_topics_and_types.get(src_topic)
            if not actual_type:
                self.get_logger().warn(f"Topic {src_topic} not found in bag, skipping")
                continue
            msg_class = self.import_msg_type(actual_type)
            if not msg_class:
                continue

            # Publisher topics are now relative; namespaces handled externally
            self.topic_publishers[src_topic] = self.create_publisher(msg_class, dst_name, 10)
            self.msg_types[src_topic] = msg_class
            self.topic_hz[src_topic] = hz
            self.last_pub_time[src_topic] = None
            self.src_to_dst[src_topic] = self.topic_publishers[src_topic].topic_name

        self.gps_marker_pub = self.create_publisher(Marker, '/gps_path_marker', 10)
        self.gps_proj = None
        self.origin_is_set = False
        if origin_lat != 0.0 and origin_lon != 0.0:
            self.get_logger().info(f"Using provided origin: LAT={origin_lat}, LON={origin_lon}")
            self.initialize_projection(origin_lat, origin_lon)

        self.message_buffer = deque()

        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_publisher = self.create_publisher(String, "status", qos_profile)
        self.topics_info_publisher = self.create_publisher(String, "topics_info", qos_profile)

        topics_json = json.dumps([
            {'name': self.src_to_dst.get(cfg['src'], cfg['src']),
             'type': all_topics_and_types.get(cfg['src'], 'unknown')}
            for dst, cfg in topic_mappings.items() if cfg['src'] in self.src_to_dst
        ])
        self.topics_info_publisher.publish(String(data=topics_json))

        self.stats_counter = {}
        self.is_playing = False

        self.status_publisher.publish(String(data='ready'))
        self.create_subscription(Empty, '/start_simulation', self.start_playback_callback, 10)
        self.get_logger().info("Bag reader is ready and waiting for /start_simulation signal.")

    def start_playback_callback(self, msg):
        if not self.is_playing:
            self.is_playing = True
            self.status_publisher.publish(String(data='playing'))
            self.get_logger().info(f"Starting real-time playback (Rate: {self.sim_rate}x)...")
            self.create_timer(1.0, self.log_stats)
            # Run play_bag_in_real_time in a thread to avoid blocking
            threading.Thread(target=self.play_bag_in_real_time, daemon=True).start()

    def _fill_buffer(self):
        if not self.reader.has_next():
            return
        count = 0
        while count < self.buffer_size and self.reader.has_next():
            topic, data, t = self.reader.read_next()
            if topic in self.topic_publishers:
                self.message_buffer.append((topic, data, t))
                count += 1

    def play_bag_in_real_time(self):
        self._fill_buffer()
        if not self.message_buffer:
            self.get_logger().info("Bag is empty or contains no relevant topics.")
            self.status_publisher.publish(String(data='finished'))
            return

        first_msg_time = self.message_buffer[0][2] / 1e9
        wall_start_time = time.time()

        while rclpy.ok():
            if not self.message_buffer:
                self._fill_buffer()
                if not self.message_buffer:
                    self.get_logger().info("End of bag file reached.")
                    self.status_publisher.publish(String(data='finished'))
                    break

            topic, data, t = self.message_buffer.popleft()
            msg_time_sec = t / 1e9
            elapsed_bag = msg_time_sec - first_msg_time
            elapsed_wall = time.time() - wall_start_time

            delay = (elapsed_bag - elapsed_wall) / self.sim_rate
            if delay > 0:
                time.sleep(delay)

            last_pub = self.last_pub_time[topic]
            min_interval = 1.0 / self.topic_hz[topic]
            if last_pub is not None and (msg_time_sec - last_pub) < min_interval:
                continue

            msg_class = self.msg_types[topic]
            msg = rclpy.serialization.deserialize_message(data, msg_class)
            self.topic_publishers[topic].publish(msg)
            self.last_pub_time[topic] = msg_time_sec

            dst_topic = self.src_to_dst[topic]
            self.stats_counter[dst_topic] = self.stats_counter.get(dst_topic, 0) + 1

            if isinstance(msg, NavSatFix):
                self.publish_gps_marker(msg)

    def initialize_projection(self, lat, lon):
        self.gps_proj = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
        local_proj_str = f"+proj=aeqd +lat_0={lat} +lon_0={lon} +datum=WGS84 +units=m +no_defs"
        self.enu_transformer = pyproj.Transformer.from_crs(
            self.gps_proj.crs,
            pyproj.CRS.from_proj4(local_proj_str),
            always_xy=True
        )
        self.origin_is_set = True
        self.get_logger().info(f"✅ GPS projection initialized to AEQD frame with origin at LAT={lat}, LON={lon}")

    def publish_gps_marker(self, gps_msg: NavSatFix):
        if not self.origin_is_set:
            if gps_msg.latitude == 0.0 or gps_msg.longitude == 0.0:
                self.get_logger().warn("Skipping GPS message with invalid coordinates (0,0) for origin initialization.")
                return
            self.initialize_projection(gps_msg.latitude, gps_msg.longitude)
            return

        try:
            x, y, z = self.enu_transformer.transform(gps_msg.longitude, gps_msg.latitude, gps_msg.altitude)
            marker = Marker()
            marker.header = gps_msg.header
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 0.8
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            self.gps_marker_pub.publish(marker)
        except Exception as e:
            self.get_logger().error(f"Failed to publish GPS marker: {e}")

    def import_msg_type(self, msg_type_str):
        try:
            parts = msg_type_str.split('/')
            if len(parts) == 3 and parts[1] == 'msg':
                pkg_name = parts[0]
                msg_name = parts[2]
            else:
                # fallback or raise error
                self.get_logger().error(f"Invalid msg_type_str format: {msg_type_str}")
                return None

            mod = importlib.import_module(f"{pkg_name}.msg")
            msg_class = getattr(mod, msg_name)
            return msg_class
        except Exception as e:
            self.get_logger().error(f"Could not import {msg_type_str}: {e}")
            return None

    def log_stats(self):
        stats_str = ", ".join([f"{k}: {v}" for k, v in self.stats_counter.items()])
        self.get_logger().info(f"Messages published so far: {stats_str}")

def main(args=None):
    rclpy.init(args=args)
    node = BagReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
