#!/usr/bin/env python3

"""
@file player_node.py
@brief ROS 2 node that listens to the /audio_data topic and plays received audio.

@details
This node subscribes to the AudioData topic and plays the raw PCM audio using sounddevice.
"""

import rclpy
from rclpy.node import Node
from audio_recorder_interfaces.msg import AudioData
import numpy as np
import sounddevice as sd
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class PlayerNode(Node):
    """
    @class PlayerNode
    @brief ROS 2 node that plays audio messages received on the /audio_data topic.
    """

    def __init__(self):
        """
        @brief Constructor for PlayerNode.
        Initializes subscription with reliable and transient-local QoS.
        """
        super().__init__('player')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.subscription = self.create_subscription(
            AudioData,
            'audio_data',
            self.audio_callback,
            qos_profile
        )
        self.get_logger().info('🎧 Player node is ready and listening to /audio_data.')

    def audio_callback(self, msg: AudioData):
        """
        @brief Callback function for received audio messages.
        Converts byte buffer to audio and plays it.

        @param msg The received AudioData message.
        """
        self.get_logger().info('🔔 Received audio_data message.')
        try:
            # Convert byte array back to int16 numpy array
            audio = np.frombuffer(msg.data, dtype=np.int16)

            self.get_logger().info(
                f'🧾 Audio length: {len(audio)}, sample rate: {msg.sample_rate}'
            )

            # Playback audio using sounddevice
            sd.play(audio, msg.sample_rate)
            sd.wait()

            self.get_logger().info('✅ Playback finished.')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to play audio: {e}')


def main(args=None):
    """
    @brief Main function that initializes and spins the PlayerNode.
    """
    rclpy.init(args=args)
    node = PlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()