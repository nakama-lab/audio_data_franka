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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import serialize_message
import rosbag2_py

class AudioRecorder(Node):
    def __init__(self):
        super().__init__('audio_recorder')
        
        # Audio configuration
        self.fs = 16000  # Standard for speech processing
        self.channels = 1  # Mono
        self.dtype = 'int16'  # Standard PCM format
        
        # File saving configuration
        self.output_dir = os.path.expanduser('~/audio_recordings')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # ROS 2 setup with QoS
        audio_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )
        self.publisher = self.create_publisher(
            UInt8MultiArray,
            '/audio_data',
            qos_profile=audio_qos
        )
        
        # Rosbag2 writer setup
        self.bag_writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri='audio_image_bag',
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.bag_writer.open(storage_options, converter_options)
        
        # Create topics in the bag
        audio_topic_info = rosbag2_py.TopicMetadata(
            name='/audio_data',
            type='std_msgs/msg/UInt8MultiArray',
            serialization_format='cdr'
        )
        image_topic_info = rosbag2_py.TopicMetadata(
            name='/image',
            type='sensor_msgs/msg/Image',
            serialization_format='cdr'
        )
        self.bag_writer.create_topic(audio_topic_info)
        self.bag_writer.create_topic(image_topic_info)
        
        # Image subscription for synchronization
        self.image_sub = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            qos_profile=10
        )
        
        self.recording = []
        self.stream = None
        self.current_filename = None
        self.last_image_msg = None

    def image_callback(self, msg):
        """Store the latest image message for synchronization"""
        self.last_image_msg = msg

    def continuous_recording(self):
        """Record continuously until Enter key is pressed"""
        self.get_logger().info("Starting continuous recording...")
        self.get_logger().info("Press Enter to stop recording...")
        
        # Set up non-blocking keyboard input
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
            
            # Create filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.current_filename = os.path.join(
                self.output_dir, 
                f"recording_{timestamp}.wav"
            )
            
            with self.stream:
                while rclpy.ok():
                    # Check for Enter key press
                    if select.select([sys.stdin], [], [], 0)[0]:
                        if sys.stdin.read(1) == '\n':
                            break
                    
                    # Process any ROS callbacks
                    rclpy.spin_once(self, timeout_sec=0.01)
            
            # Save recording when loop exits
            self.save_recording()
            
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def audio_callback(self, indata, frames, time, status):
        """Callback for audio data"""
        self.recording.extend(indata.copy())
        
        # Create and publish audio message
        audio_msg = UInt8MultiArray()
        audio_msg.data = indata.tobytes()
        self.publisher.publish(audio_msg)
        
        # Write to rosbag with synchronized image if available
        if self.last_image_msg:
            audio_stamp = self.get_clock().now().to_msg()
            audio_msg.header.stamp = audio_stamp
            
            # Write synchronized messages to bag
            self.bag_writer.write(
                '/audio_data',
                serialize_message(audio_msg),
                audio_stamp
            )
            self.bag_writer.write(
                '/image',
                serialize_message(self.last_image_msg),
                audio_stamp
            )

    def save_recording(self):
        """Save buffered audio to file"""
        if len(self.recording) == 0:
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
        """Clean up resources"""
        if hasattr(self, 'stream') and self.stream is not None:
            self.stream.close()
        self.bag_writer.close()
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
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore if already shutdown

if __name__ == '__main__':
    main()