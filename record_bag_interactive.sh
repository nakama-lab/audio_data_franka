#!/bin/bash
set -e

# Source the ROS 2 workspace
source install/setup.bash

# Create output directory
mkdir -p rosbags

# Generate timestamped bag name
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="rosbag_$TIMESTAMP"
BAG_PATH="./rosbags/$BAG_NAME"
QOS_OVERRIDE_PATH="qos_override.yaml"

echo "🎙️ Will record to: $BAG_PATH"
echo "📄 Using QoS overrides from: $QOS_OVERRIDE_PATH"

# Topics we want to record
REQUIRED_TOPICS=(
  "/audio_data"
  "/franka_robot_state_broadcaster/robot_state"
)

# Wait until all required topics are available
echo "⏳ Waiting for required topics to become available..."
for TOPIC in "${REQUIRED_TOPICS[@]}"; do
  echo "⏳ Waiting for topic with data: $TOPIC"
  until timeout 3s ros2 topic echo "$TOPIC" --qos-reliability reliable --qos-durability transient_local --once > /dev/null 2>&1; do
    echo "🔄 Still waiting for message on $TOPIC..."
    sleep 1
  done
  echo "✅ Message received from $TOPIC"
done


# Start rosbag in background
echo "🎥 Starting rosbag recording to: $BAG_PATH"
ros2 bag record -o "$BAG_PATH" "${REQUIRED_TOPICS[@]}" \
  --qos-profile-overrides-path "$QOS_OVERRIDE_PATH" &
BAG_PID=$!
sleep 2  # Let rosbag initialize fully

# Begin audio recording
echo "🔴 Starting audio recording (press ENTER in terminal when done)..."
ros2 service call /start_recording audio_recorder_interfaces/srv/StartRecording "{}"

# Wait for user to press Enter
read -r

# Stop rosbag safely
echo "🛑 Stopping rosbag..."
if ps -p $BAG_PID > /dev/null; then
  kill -SIGINT "$BAG_PID"
  wait "$BAG_PID"
  echo "✅ Rosbag stopped."
else
  echo "⚠️ Rosbag process already exited or not found."
fi
