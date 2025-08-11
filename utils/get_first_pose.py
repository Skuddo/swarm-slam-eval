#!/usr/bin/env python3
import sys
import argparse
import rclpy
import rclpy.serialization
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry

def get_first_pose(bag_path: str, ground_truth_topic: str):
    """
    Reads a bag file and prints the x,y coordinates of the first message
    on the specified ground_truth_topic.
    """
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    # Get the message type for deserialization
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if ground_truth_topic not in topic_types:
        print(f"Error: Topic '{ground_truth_topic}' not found in bag.", file=sys.stderr)
        return

    # We only expect Odometry messages for ground truth based on your setup
    msg_type_str = topic_types[ground_truth_topic]
    if 'Odometry' not in msg_type_str:
        print(f"Error: Topic is not of type Odometry, got {msg_type_str}", file=sys.stderr)
        return
    
    # Read messages until we find the first one on our target topic
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == ground_truth_topic:
            msg = rclpy.serialization.deserialize_message(data, Odometry)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            # Print coordinates in a parseable format and exit
            print(f"{x},{y}")
            return

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract first pose from a ROS2 bag.')
    parser.add_argument('--bag-path', required=True, help='Path to the bag file directory.')
    parser.add_argument('--topic', required=True, help='The ground truth topic to search for.')
    args = parser.parse_args()
    
    get_first_pose(args.bag_path, args.topic)