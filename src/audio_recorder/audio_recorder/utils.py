from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    LivelinessPolicy,
    QoSDurabilityPolicy
)
import rclpy.duration

def get_audio_qos():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Ensure delivery
        history=QoSHistoryPolicy.KEEP_LAST,         # Keep recent samples
        depth=10,                                   # Buffer size
        deadline=rclpy.duration.Duration(seconds=0.1), # Max delay
        liveliness=LivelinessPolicy.AUTOMATIC,  # Automatic liveliness
        liveliness_lease_duration=rclpy.duration.Duration(seconds=1)
    )