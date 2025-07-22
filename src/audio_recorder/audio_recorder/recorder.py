#!/usr/bin/env python3

"""
@file recorder_node.py
@brief ROS 2 node for recording and replaying audio using services and publishing AudioData.

@details
 - Provides two services:
    1. start_recording (audio_recorder_interfaces/srv/StartRecording)
    2. play_audio (audio_recorder_interfaces/srv/PlayAudio)
 - Records audio from a microphone using sounddevice and saves it as a WAV file.
 - Publishes the audio data as a single AudioData message upon recording completion.
"""

import rclpy
from rclpy.node import Node
from audio_recorder_interfaces.srv import StartRecording, PlayAudio
from audio_recorder_interfaces.msg import AudioData
import numpy as np
import sounddevice as sd
import soundfile as sf
import os
import threading
from datetime import datetime
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# Constants for recording parameters
SAMPLE_RATE = 44100
CHANNELS = 1
DTYPE = 'int16'

class RecorderNode(Node):
    """
    @class RecorderNode
    @brief A ROS 2 node that records audio input and publishes the result.

    This node implements services to start recording audio and to replay the latest recorded file.
    The recorded audio is published as an AudioData message.
    """

    def __init__(self):
        """
        @brief Constructor for RecorderNode.
        Initializes services and publisher with appropriate QoS.
        """
        super().__init__('recorder')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.srv_record = self.create_service(
            StartRecording, 'start_recording', self.handle_record
        )
        self.srv_play = self.create_service(
            PlayAudio, 'play_audio', self.handle_play
        )

        self.publisher_ = self.create_publisher(AudioData, 'audio_data', qos_profile)

        self.get_logger().info('🎙️ Recorder node is running!')

    def handle_record(self, request, response):
        """
        @brief Handles the start_recording service call.
        Records audio until ENTER is pressed and publishes the result.

        @param request The service request (unused).
        @param response The service response indicating success or failure.

        @return Modified service response.
        """
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        audio_file = f'recorded_audio_{timestamp_str}.wav'
        debug_file = 'debug_audio.wav'
        os.makedirs('rosbags', exist_ok=True)

        self.get_logger().info(f'⏺️ Recording to {audio_file}... Press ENTER to stop.')

        stop_event = threading.Event()
        recorded_frames = []

        def listen_for_stop():
            input()
            stop_event.set()

        def callback(indata, frames, time, status):
            if stop_event.is_set():
                raise sd.CallbackStop()
            recorded_frames.append(indata.copy())

        threading.Thread(target=listen_for_stop, daemon=True).start()

        try:
            with sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, dtype=DTYPE, callback=callback):
                while not stop_event.is_set():
                    sd.sleep(100)

            audio = np.concatenate(recorded_frames).flatten()

            self.get_logger().info(
                f'📊 Recorded audio stats: dtype={audio.dtype}, '
                f'min={audio.min()}, max={audio.max()}, samples={len(audio)}'
            )

            # Save debug and timestamped audio files
            sf.write(audio_file, audio, SAMPLE_RATE)
            sf.write(debug_file, audio, SAMPLE_RATE)
            self.get_logger().info(f'💾 Saved debug audio to {debug_file}')

            # Publish AudioData message
            msg = AudioData()
            msg.stamp = self.get_clock().now().to_msg()
            msg.sample_rate = SAMPLE_RATE
            msg.channels = CHANNELS
            msg.sample_width = 2  # int16 = 2 bytes
            msg.data = audio.tobytes()
            self.publisher_.publish(msg)
            self.get_logger().info('📤 Published final audio_data message.')

            response.success = True
        except Exception as e:
            self.get_logger().error(f'❌ Failed to record: {e}')
            response.success = False

        return response

    def handle_play(self, request, response):
        """
        @brief Handles the play_audio service call.
        Plays back the most recently recorded audio file.

        @param request The service request (unused).
        @param response The service response indicating success or failure.

        @return Modified service response.
        """
        try:
            wav_files = [f for f in os.listdir('.') if f.startswith('recorded_audio_') and f.endswith('.wav')]
            if not wav_files:
                raise FileNotFoundError("No audio files found.")

            latest_file = max(wav_files, key=os.path.getctime)
            self.get_logger().info(f'▶️ Playing {latest_file}')
            data, fs = sf.read(latest_file, dtype='float32')
            sd.play(data, fs)
            sd.wait()
            response.success = True
        except Exception as e:
            self.get_logger().error(f'❌ Failed to play file: {e}')
            response.success = False

        return response


def main(args=None):
    """
    @brief Main function that initializes and spins the RecorderNode.
    """
    rclpy.init(args=args)
    node = RecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()