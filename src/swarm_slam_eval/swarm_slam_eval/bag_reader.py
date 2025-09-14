#!/usr/bin/env python3
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
        self.declare_parameter('robot_prefix', '')

        bag_path = self.get_parameter('bag_path').get_parameter_value().string_value
        self.is_clock_publisher = self.get_parameter('is_clock_publisher').get_parameter_value().bool_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value
        self.robot_prefix = self.get_parameter('robot_prefix').get_parameter_value().string_value

        if not bag_path:
            self.get_logger().error("'bag_path' parameter not set. Shutting down.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Loading bag from: {bag_path}")

        # Look for config.yaml next to bag; if not there try dataset folder (one level up)
        config_path = os.path.join(os.path.dirname(bag_path), 'config.yaml')
        if not os.path.exists(config_path):
            config_path = os.path.join(os.path.dirname(os.path.dirname(bag_path)), 'config.yaml')

        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            # Try 'ground' key first, then fall back to first top-level key (works for S3E)
            if 'ground' in config:
                topic_mappings = config.get('ground', {}).get('topics', {})
            else:
                first_key = next(iter(config))
                topic_mappings = config.get(first_key, {}).get('topics', {})
        except (IOError, yaml.YAMLError, StopIteration) as e:
            self.get_logger().error(f"Failed to load config file at {config_path}: {e}")
            rclpy.shutdown()
            return

        # open the bag
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
        self.src_to_dst = {}
        all_topics_and_types = {t.name: t.type for t in self.reader.get_all_topics_and_types()}

        # Build publishers based on config topics; support {robot} placeholder
        for dst_name, cfg in topic_mappings.items():
            if not isinstance(cfg, dict):
                continue
            src_template = cfg.get('src')
            if not src_template:
                continue

            if self.robot_prefix:
                try:
                    src_topic = src_template.format(robot=self.robot_prefix)
                except Exception:
                    src_topic = src_template
            else:
                src_topic = src_template

            actual_type = all_topics_and_types.get(src_topic)
            if not actual_type:
                self.get_logger().warn(f"Topic '{src_topic}' not found in bag, skipping.")
                continue

            msg_class = self._import_msg_type(actual_type)
            if msg_class:
                # create publisher on dst_name (this is what downstream nodes will subscribe to)
                pub = self.create_publisher(msg_class, dst_name, DATA_QOS)
                # key publishers by source topic from bag so we know which incoming messages to forward
                self.topic_publishers[src_topic] = pub
                self.msg_types[src_topic] = msg_class
                self.src_to_dst[src_topic] = dst_name

        # Publish topics_info in the format downstream nodes expect: {'name': <dst>, 'type': <type>}
        # also include 'src' (resolved source in bag) as extra helpful info
        topics_info = []
        for dst, cfg in topic_mappings.items():
            if not isinstance(cfg, dict):
                continue
            src_template = cfg.get('src', '')
            # resolved source (if robot_prefix used)
            try:
                resolved_src = src_template.format(robot=self.robot_prefix) if self.robot_prefix and '{robot' in src_template else src_template
            except Exception:
                resolved_src = src_template
            # only include if the resolved src is actually present and we created a publisher for it
            if resolved_src in self.topic_publishers:
                topics_info.append({
                    'name': dst,  # destination topic name (what other nodes should subscribe to)
                    'type': all_topics_and_types.get(resolved_src, 'unknown'),
                    'src': resolved_src
                })

        topics_json = json.dumps(topics_info)
        self.topics_info_publisher = self.create_publisher(String, 'topics_info', SIGNAL_QOS)
        # publish once to announce available topics
        self.topics_info_publisher.publish(String(data=topics_json))
        self.get_logger().info(f"Published topics_info: {topics_json}")

        # clock publisher (only one bag_reader should publish /clock)
        self.clock_publisher = None
        if self.is_clock_publisher:
            self.clock_publisher = self.create_publisher(Clock, '/clock', CLOCK_QOS)

        # Determine canonical start time by peeking into bag
        self.canonical_start_time_ns = -1
        try:
            peek_reader = SequentialReader()
            peek_reader.open(self.storage_options, self.converter_options)
            if peek_reader.has_next():
                _, _, t = peek_reader.read_next()
                self.canonical_start_time_ns = t
                self.get_logger().info(f"Canonical start time: {self.canonical_start_time_ns / 1e9:.6f}")

                if self.is_clock_publisher and self.clock_publisher:
                    initial_clock = Clock()
                    sim_time = rclpy.time.Time(nanoseconds=0)
                    initial_clock.clock = sim_time.to_msg()
                    self.clock_publisher.publish(initial_clock)
                    self.get_logger().info("Published initial preemptive /clock")
        except Exception as e:
            self.get_logger().error(f"Failed to pre-scan bag for initial timestamp: {e}")

        self.message_buffer = deque()
        self.is_playing = False

        self.status_publisher = self.create_publisher(String, 'status', SIGNAL_QOS)
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

            msg_class = self.msg_types.get(topic)
            if msg_class is None:
                # unexpected: no registered type
                continue

            try:
                msg = rclpy.serialization.deserialize_message(data, msg_class)
            except Exception as e:
                self.get_logger().error(f"Deserialization failed on {topic}: {e}")
                continue

            # update header stamps if present
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                old_frame = getattr(msg.header, 'frame_id', 'world') or 'world'
                new_stamp = Time()
                new_stamp.sec = int(rebased_ns / 1e9)
                new_stamp.nanosec = int(rebased_ns % 1e9)
                msg.header.stamp = new_stamp
                msg.header.frame_id = old_frame

            # publish using the publisher keyed by src topic
            self.topic_publishers[topic].publish(msg)

            if self.is_clock_publisher and self.clock_publisher:
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
