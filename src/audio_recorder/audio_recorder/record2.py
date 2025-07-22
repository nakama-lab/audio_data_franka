#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sounddevice as sd
import numpy as np
from std_msgs.msg import UInt8MultiArray
from .utils import get_audio_qos
from builtin_interfaces.msg import Time

class AudioRecorder(Node):
    def __init__(self):
        super().__init__('audio_recorder')
        
        # Configure QoS for audio streaming
        qos_profile = get_audio_qos()
        
        self.publisher = self.create_publisher(
            UInt8MultiArray, 
            '/audio_data', 
            qos_profile=qos_profile
        )
        
        # Audio configuration
        self.fs = 44100  # Sample rate
        self.chunk_size = 1024  # Samples per chunk
        self.is_recording = False

    def start_recording(self):
        self.is_recording = True
        self.get_logger().info("Starting audio stream...")
        
        def callback(indata, frames, time, status):
            if not self.is_recording:
                raise sd.CallbackStop
            
            # Convert and publish
            audio_bytes = (indata * 32767).astype(np.int16).tobytes()
            msg = UInt8MultiArray(data=list(audio_bytes))
            self.publisher.publish(msg)
        
        # Stream audio with callback
        with sd.InputStream(
            samplerate=self.fs,
            channels=1,
            callback=callback,
            blocksize=self.chunk_size
        ):
            while self.is_recording:
                rclpy.spin_once(self, timeout_sec=0.1)

    def stop_recording(self):
        self.is_recording = False
        self.get_logger().info("Stopped audio stream")

def main(args=None):
    rclpy.init(args=args)
    node = AudioRecorder()
    
    try:
        node.start_recording()
    except KeyboardInterrupt:
        node.stop_recording()
    
    node.destroy_node()
    rclpy.shutdown()