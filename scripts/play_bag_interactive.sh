#!/bin/bash
set -eo pipefail
source install/setup.bash
set -u

BAG_DIR="rosbags"
QOS_OVERRIDE="qos_override.yaml"

if [[ ! -f "$QOS_OVERRIDE" ]]; then
  echo "❌ QoS override file '$QOS_OVERRIDE' not found!"
  exit 1
fi

if [[ ! -d "$BAG_DIR" ]]; then
  echo "❌ Bag directory '$BAG_DIR' not found!"
  exit 1
fi

bags=("$BAG_DIR"/rosbag_*)
bag_count="${#bags[@]}"

if (( bag_count == 0 )); then
  echo "❌ No bags found in '$BAG_DIR'."
  exit 1
fi

echo "📦 Found $bag_count bag(s):"
for i in "${!bags[@]}"; do
  echo "[$i] $(basename "${bags[$i]}")"
done

read -p "▶️  Enter the number of the bag to preview and play: " index
if ! [[ "$index" =~ ^[0-9]+$ ]] || (( index < 0 || index >= bag_count )); then
  echo "❌ Invalid index: $index"
  exit 1
fi

BAG_PATH="${bags[$index]}"

echo "🔍 Previewing: $BAG_PATH"
echo "-------------------------------------------"
ros2 bag info "$BAG_PATH" || { echo "❌ Failed to preview bag info."; exit 1; }

DB_PATH=$(find "$BAG_PATH" -name "*.db3" | head -n1)
if [[ -z "$DB_PATH" ]]; then
  echo "❌ No .db3 file found in $BAG_PATH"
  exit 1
fi

echo ""
echo "📁 Bag: $BAG_PATH"
echo "📝 Available topics:"
TOPICS=()
while IFS= read -r topic; do
  TOPICS+=("$topic")
done < <(sqlite3 "$DB_PATH" "SELECT DISTINCT name FROM topics;")

for i in "${!TOPICS[@]}"; do
  echo "[$i] ${TOPICS[$i]}"
done

read -p "🔁 Do you want to loop playback? (y/n): " loop_response
LOOP=false
if [[ "$loop_response" == "y" || "$loop_response" == "Y" ]]; then
  LOOP=true
fi

echo ""
read -p "🎯 Enter the indices of topics to play (space-separated, e.g. '0 2'): " -a selected_indices

SELECTED_TOPICS=()
for i in "${selected_indices[@]}"; do
  if [[ "$i" =~ ^[0-9]+$ ]] && (( i >= 0 && i < ${#TOPICS[@]} )); then
    SELECTED_TOPICS+=("${TOPICS[$i]}")
  else
    echo "⚠️  Skipping invalid index: $i"
  fi
done

if (( ${#SELECTED_TOPICS[@]} == 0 )); then
  echo "❌ No valid topics selected. Exiting."
  exit 1
fi

echo ""
echo "📡 Selected topics:"
for topic in "${SELECTED_TOPICS[@]}"; do
  echo "  • $topic"
done

echo ""
read -p "👀 Inspect a few messages before playback? (y/n): " inspect
if [[ "$inspect" == "y" || "$inspect" == "Y" ]]; then
  for topic in "${SELECTED_TOPICS[@]}"; do
    echo ""
    echo "🔸 Inspecting topic: $topic"

    ros2 bag play "$BAG_PATH" --topics "$topic" \
      --qos-profile-overrides-path "$QOS_OVERRIDE" &
    BAG_PID=$!

    sleep 1.5

    if timeout 4s ros2 topic echo "$topic" \
      --qos-durability transient_local \
      --qos-reliability reliable \
      --once; then
      echo "✅ Successfully echoed message from $topic"
    else
      echo "⚠️  No message received from $topic (might be due to QoS mismatch)"
    fi

    kill "$BAG_PID" 2>/dev/null || true
    sleep 0.5
  done
fi

echo ""

if $LOOP; then
  echo "🔁 Starting looped playback. Press Ctrl+C to stop."

  trap 'echo -e "\n🛑 Playback stopped."; exit 0' SIGINT

  while true; do
    ros2 bag play "$BAG_PATH" --topics "${SELECTED_TOPICS[@]}" \
      --qos-profile-overrides-path "$QOS_OVERRIDE"
    echo "🔁 Loop complete. Restarting in 1s..."
    sleep 1
  done
else
  echo "▶️ Playing once..."
  ros2 bag play "$BAG_PATH" --topics "${SELECTED_TOPICS[@]}" \
    --qos-profile-overrides-path "$QOS_OVERRIDE"
fi
