# utils.py

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

def get_audio_qos():
    """
    Returns a QoSProfile suitable for audio transmission and rosbag compatibility.
    """
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,   # Use BEST_EFFORT for lower latency, RELIABLE for guaranteed delivery
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )
