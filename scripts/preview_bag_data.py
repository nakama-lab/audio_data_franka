#!/usr/bin/env python3

import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rclpy

def preview_bag(bag_path):
    rclpy.init()

    print(f"📁 Bag: {bag_path}")

    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    topics = [t.name for t in topic_types]
    print("📝 Topics:", topics)

    print(f"{'Timestamp':<26} | {'Topic':<30}")
    print("-" * 60)

    count = 0
    while reader.has_next() and count < 10:
        topic, data, timestamp = reader.read_next()
        ts_sec = timestamp / 1e9
        print(f"{ts_sec:<26.9f} | {topic}")
        count += 1

    rclpy.shutdown()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: preview_bag_data.py <bag_folder>")
        sys.exit(1)
    preview_bag(sys.argv[1])
