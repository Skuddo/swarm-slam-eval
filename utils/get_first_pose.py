import sys
import argparse
import rclpy
import rclpy.serialization
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

def get_first_pose(bag_path: str, ground_truth_topic: str):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if ground_truth_topic not in topic_types:
        print(f"Error: Topic '{ground_truth_topic}' not found in bag.", file=sys.stderr)
        return

    msg_type_str = topic_types[ground_truth_topic]
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == ground_truth_topic:
            if 'Odometry' in msg_type_str:
                msg = rclpy.serialization.deserialize_message(data, Odometry)
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                print(f"{x},{y}")
                return
            elif 'NavSatFix' in msg_type_str:
                msg = rclpy.serialization.deserialize_message(data, NavSatFix)
                latitude = msg.latitude
                longitude = msg.longitude
                print(f"{latitude},{longitude}")
                return
            else:
                print(f"Error: Topic is not of a supported type (Odometry or NavSatFix), got {msg_type_str}", file=sys.stderr)
                return

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract first pose from a ROS2 bag.')
    parser.add_argument('--bag-path', required=True, help='Path to the bag file directory.')
    parser.add_argument('--topic', required=True, help='The ground truth topic to search for.')
    args = parser.parse_args()
    
    get_first_pose(args.bag_path, args.topic)