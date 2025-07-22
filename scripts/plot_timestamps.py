import sqlite3
import os
import numpy as np
import matplotlib.pyplot as plt

# Set this to your actual bag path
BAG_PATH = "rosbags/rosbag_20250718_172140"
DB3_FILE = next(
    (f for f in os.listdir(BAG_PATH) if f.endswith(".db3")), None
)
if not DB3_FILE:
    raise FileNotFoundError("No .db3 file found in bag folder.")

DB_PATH = os.path.join(BAG_PATH, DB3_FILE)
print(f"📦 Loading bag from: {DB_PATH}")

# Connect to the SQLite DB
conn = sqlite3.connect(DB_PATH)
cursor = conn.cursor()

# Get topic IDs
cursor.execute("SELECT id, name FROM topics")
topics = dict(cursor.fetchall())
print(f"📋 Topics: {topics}")

# Select only the topics we're interested in
topic_names = ["/audio_data", "/franka_robot_state_broadcaster/robot_state"]
topic_ids = {name: id for id, name in topics.items() if name in topic_names}

if len(topic_ids) < 2:
    raise ValueError("❌ Missing expected topics in the bag!")

# Extract timestamps for each topic
def get_timestamps(topic_id):
    cursor.execute("SELECT timestamp FROM messages WHERE topic_id=?", (topic_id,))
    return np.array([row[0] for row in cursor.fetchall()]) * 1e-9  # ns to sec

timestamps_audio = get_timestamps(topic_ids["/audio_data"])
timestamps_robot = get_timestamps(topic_ids["/franka_robot_state_broadcaster/robot_state"])

conn.close()

# Normalize to start at zero
t0 = min(timestamps_audio[0], timestamps_robot[0])
timestamps_audio -= t0
timestamps_robot -= t0

# Plotting
plt.figure(figsize=(10, 5))
plt.plot(timestamps_audio, np.zeros_like(timestamps_audio), 'ro', label='audio_data')
plt.plot(timestamps_robot, np.ones_like(timestamps_robot), 'bo', label='robot_state')
plt.yticks([0, 1], ['audio_data', 'robot_state'])
plt.xlabel("Time (s)")
plt.title("🕒 Message Timestamps: audio vs robot_state")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
