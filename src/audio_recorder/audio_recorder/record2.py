#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sounddevice as sd
import soundfile as sf
import numpy as np
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Image
from datetime import datetime
import os
import sys
import select
import tty
import termios
from .utils import get_audio_qos
from rclpy.serialization import serialize_message
import rosbag2_py

class AudioRecorder(Node):
    def __init__(self):
        super().__init__('audio_recorder')

        # Audio configuration
        self.fs = 16000
        self.channels = 1
        self.dtype = 'int16'

        # File saving configuration
        self.output_dir = os.path.expanduser('~/audio_recordings')
        os.makedirs(self.output_dir, exist_ok=True)

        # Rosbag directory with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.bag_dir = f'audio_image_bag_{timestamp}'

        # QoS profile setup
        qos_profile = get_audio_qos()

        self.publisher = self.create_publisher(
            UInt8MultiArray,
            '/audio_data',
            qos_profile=qos_profile
        )

        # Rosbag2 writer setup
        self.bag_writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_dir,
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.bag_writer.open(storage_options, converter_options)

        # Create topics in the bag
        self.bag_writer.create_topic(rosbag2_py.TopicMetadata(
            name='/audio_data',
            type='std_msgs/msg/UInt8MultiArray',
            serialization_format='cdr'
        ))
        self.bag_writer.create_topic(rosbag2_py.TopicMetadata(
            name='/image',
            type='sensor_msgs/msg/Image',
            serialization_format='cdr'
        ))

        self.image_sub = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            qos_profile=qos_profile
        )

        self.recording = []
        self.stream = None
        self.current_filename = None
        self.last_image_msg = None

    def image_callback(self, msg):
        self.last_image_msg = msg

    def continuous_recording(self):
        self.get_logger().info("Starting continuous recording...")
        self.get_logger().info("Press Enter to stop recording...")

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())

            # Start audio stream
            self.stream = sd.InputStream(
                samplerate=self.fs,
                channels=self.channels,
                dtype=self.dtype,
                callback=self.audio_callback
            )

            # Create unique filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.current_filename = os.path.join(
                self.output_dir,
                f"recording_{timestamp}.wav"
            )

            with self.stream:
                while rclpy.ok():
                    if select.select([sys.stdin], [], [], 0)[0]:
                        if sys.stdin.read(1) == '\n':
                            break
                    rclpy.spin_once(self, timeout_sec=0.01)

            self.save_recording()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def audio_callback(self, indata, frames, time, status):
        self.recording.extend(indata.copy())

        # Create and publish ROS message
        audio_msg = UInt8MultiArray()
        audio_msg.data = indata.tobytes()
        self.publisher.publish(audio_msg)

        # Timestamp as int (nanoseconds)
        stamp = self.get_clock().now().to_msg()
        timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec

        # Write audio message to bag
        self.bag_writer.write(
            '/audio_data',
            serialize_message(audio_msg),
            timestamp_ns
        )

        # Write last image (if exists) with same timestamp
        if self.last_image_msg:
            self.bag_writer.write(
                '/image',
                serialize_message(self.last_image_msg),
                timestamp_ns
    )


    def save_recording(self):
        if not self.recording:
            self.get_logger().warning("No audio data to save")
            return

        recording = np.concatenate(self.recording)
        sf.write(
            self.current_filename,
            recording,
            self.fs,
            subtype='PCM_16'
        )
        self.get_logger().info(f"Saved recording to {self.current_filename}")
        self.recording.clear()

    def destroy_node(self):
        if hasattr(self, 'stream') and self.stream is not None:
            self.stream.close()
        # Removed .close() call to avoid AttributeError
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = AudioRecorder()

    try:
        recorder.continuous_recording()
    except KeyboardInterrupt:
        recorder.get_logger().info("Shutting down recorder...")
    except Exception as e:
        recorder.get_logger().error(f"Error: {str(e)}")
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
