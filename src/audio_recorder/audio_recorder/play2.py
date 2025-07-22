#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sounddevice as sd
import numpy as np
from std_msgs.msg import UInt8MultiArray
from .utils import get_audio_qos

class AudioPlayer(Node):
    def __init__(self):
        super().__init__('audio_player')
        
        # Initialize stream as None first
        self.stream = None
        
        try:
            # Configure QoS
            qos_profile = get_audio_qos()
            
            self.subscription = self.create_subscription(
                UInt8MultiArray,
                '/audio_data',
                self.listener_callback,
                qos_profile=qos_profile
            )
            
            # Audio configuration
            self.fs = 44100
            self.stream = sd.OutputStream(
                samplerate=self.fs,
                channels=1,
                dtype='float32'
            )
            self.stream.start()
            self.get_logger().info("Audio player initialized")
            
        except Exception as e:
            self.get_logger().error(f"Initialization failed: {str(e)}")
            raise

    def listener_callback(self, msg):
        if not hasattr(self, 'stream') or self.stream is None:
            self.get_logger().error("Audio stream not initialized")
            return
            
        try:
            audio_data = np.frombuffer(bytes(msg.data), dtype=np.int16)
            audio_data = audio_data.astype(np.float32) / 32767.0
            self.stream.write(audio_data)
        except Exception as e:
            self.get_logger().error(f"Playback error: {str(e)}")

    def destroy_node(self):
        """Cleanup before node destruction"""
        if hasattr(self, 'stream') and self.stream is not None:
            self.stream.stop()
            self.stream.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down player...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()