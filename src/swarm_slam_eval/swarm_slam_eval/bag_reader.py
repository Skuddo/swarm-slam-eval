import importlib
import json
import yaml
import os
import time
import threading
from collections import deque

import rclpy
import rclpy.serialization
from rclpy.node import Node
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

from std_srvs.srv import Trigger
from std_msgs.msg import String, Empty
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

from swarm_slam_eval.qos_profiles import DATA_QOS, SIGNAL_QOS, CLOCK_QOS


class BagReaderNode(Node):
    def __init__(self):
        super().__init__('bag_reader_node')

        self.declare_parameter('bag_path', '')
        self.declare_parameter('is_clock_publisher', False)
        self.declare_parameter('buffer_size', 200)

        bag_path = self.get_parameter('bag_path').get_parameter_value().string_value
        self.is_clock_publisher = self.get_parameter('is_clock_publisher').get_parameter_value().bool_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value

        if not bag_path:
            self.get_logger().error("'bag_path' parameter not set. Shutting down.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Loading bag from: {bag_path}")

        config_path = os.path.join(os.path.dirname(bag_path), 'config.yaml')
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            topic_mappings = config.get('ground', {}).get('topics', {})
        except (IOError, yaml.YAMLError) as e:
            self.get_logger().error(f"Failed to load config file at {config_path}: {e}")
            rclpy.shutdown()
            return

        self.storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        self.converter_options = ConverterOptions('', '')
        self.reader = SequentialReader()
        try:
            self.reader.open(self.storage_options, self.converter_options)
        except Exception as e:
            self.get_logger().error(f"Failed to open bag file: {e}")
            rclpy.shutdown()
            return

        self.topic_publishers = {}
        self.msg_types = {}
        all_topics_and_types = {t.name: t.type for t in self.reader.get_all_topics_and_types()}
        self.src_to_dst = {}

        for dst_name, cfg in topic_mappings.items():
            src_topic = cfg['src']
            actual_type = all_topics_and_types.get(src_topic)
            if not actual_type:
                self.get_logger().warn(f"Topic '{src_topic}' not found in bag, skipping.")
                continue

            msg_class = self._import_msg_type(actual_type)
            if msg_class:
                pub = self.create_publisher(msg_class, dst_name, DATA_QOS)
                self.topic_publishers[src_topic] = pub
                self.msg_types[src_topic] = msg_class
                self.src_to_dst[src_topic] = dst_name

        topics_json = json.dumps([
            {'name': dst, 'type': all_topics_and_types.get(cfg['src'], 'unknown')}
            for dst, cfg in topic_mappings.items() if cfg['src'] in self.src_to_dst
        ])
        self.topics_info_publisher = self.create_publisher(String, "topics_info", SIGNAL_QOS)
        self.topics_info_publisher.publish(String(data=topics_json))
        self.get_logger().info(f"Published topics_info: {topics_json}")

        self.clock_publisher = None
        if self.is_clock_publisher:
            self.clock_publisher = self.create_publisher(Clock, '/clock', CLOCK_QOS)

        self.canonical_start_time_ns = -1
        try:
            peek_reader = SequentialReader()
            peek_reader.open(self.storage_options, self.converter_options)
            if peek_reader.has_next():
                _, _, t = peek_reader.read_next()
                self.canonical_start_time_ns = t
                self.get_logger().info(f"Canonical start time: {self.canonical_start_time_ns / 1e9:.6f}")

                if self.is_clock_publisher:
                    initial_clock = Clock()
                    sim_time = rclpy.time.Time(nanoseconds=0)
                    initial_clock.clock = sim_time.to_msg()
                    self.clock_publisher.publish(initial_clock)
                    self.get_logger().info("Published initial preemptive /clock")
        except Exception as e:
            self.get_logger().error(f"Failed to pre-scan bag for initial timestamp: {e}")

        self.message_buffer = deque()
        self.is_playing = False

        self.status_publisher = self.create_publisher(String, "status", SIGNAL_QOS)
        self.status_publisher.publish(String(data='ready'))
        self.create_subscription(Empty, '/start_simulation', self.start_playback_callback, SIGNAL_QOS)

        self.reg_client = self.create_client(Trigger, '/register_ready')
        self._register_self()

        self.get_logger().info("Bag reader is ready and waiting for /start_simulation signal.")

    def _register_self(self):
        def call_service_thread():
            while not self.reg_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info("Sync service '/register_ready' not available, waiting...")
            future = self.reg_client.call_async(Trigger.Request())
            self.get_logger().info("Registration request sent to sync node.")
        threading.Thread(target=call_service_thread, daemon=True).start()

    def start_playback_callback(self, msg):
        if not self.is_playing:
            self.is_playing = True
            self.status_publisher.publish(String(data='playing'))
            threading.Thread(target=self.play_bag_in_real_time, daemon=True).start()

    def _fill_buffer(self):
        count = 0
        while count < self.buffer_size and self.reader.has_next():
            topic, data, t = self.reader.read_next()
            if topic in self.topic_publishers:
                self.message_buffer.append((topic, data, t))
                count += 1

    def play_bag_in_real_time(self):
        if self.canonical_start_time_ns == -1:
            self.get_logger().error("Canonical start time not set. Playback aborted.")
            return

        self._fill_buffer()
        if not self.message_buffer:
            self.get_logger().info("Bag empty or no relevant topics.")
            self.status_publisher.publish(String(data='finished'))
            return

        wall_start_time = time.time()

        while rclpy.ok():
            if not self.message_buffer:
                self._fill_buffer()
                if not self.message_buffer:
                    self.get_logger().info("End of bag reached.")
                    self.status_publisher.publish(String(data='finished'))
                    break

            topic, data, t_ns = self.message_buffer.popleft()
            rebased_ns = t_ns - self.canonical_start_time_ns
            elapsed_wall = time.time() - wall_start_time
            delay_s = (rebased_ns / 1e9) - elapsed_wall
            if delay_s > 0:
                time.sleep(delay_s)

            msg_class = self.msg_types[topic]
            try:
                msg = rclpy.serialization.deserialize_message(data, msg_class)
            except Exception as e:
                self.get_logger().error(f"Deserialization failed on {topic}: {e}")
                continue

            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                old_frame = msg.header.frame_id or "world"
                new_stamp = Time()
                new_stamp.sec = int(rebased_ns / 1e9)
                new_stamp.nanosec = int(rebased_ns % 1e9)
                msg.header.stamp = new_stamp
                msg.header.frame_id = old_frame

            self.topic_publishers[topic].publish(msg)

            if self.is_clock_publisher:
                clock_msg = Clock()
                sim_time = rclpy.time.Time(nanoseconds=rebased_ns)
                clock_msg.clock = sim_time.to_msg()
                self.clock_publisher.publish(clock_msg)

            if len(self.message_buffer) < self.buffer_size / 2:
                self._fill_buffer()

    def _import_msg_type(self, msg_type_str):
        try:
            parts = msg_type_str.split('/')
            if len(parts) == 3:
                pkg_name, msg_name = parts[0], parts[2]
            elif len(parts) == 2:
                pkg_name, msg_name = parts
            else:
                self.get_logger().error(f"Invalid msg_type_str: {msg_type_str}")
                return None
            mod = importlib.import_module(f"{pkg_name}.msg")
            return getattr(mod, msg_name)
        except Exception as e:
            self.get_logger().error(f"Cannot import '{msg_type_str}': {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = BagReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
