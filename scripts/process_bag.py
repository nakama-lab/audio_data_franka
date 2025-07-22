#!/usr/bin/env python3
import os
import json
import numpy as np
from pydub import AudioSegment
from pydub.effects import normalize
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
import whisper

# === CONFIGURATION ===
ROSBAG_DIR = "rosbags"
AUDIO_TOPIC = "/audio_data"
ROBOT_TOPIC = "/franka_robot_state_broadcaster/robot_state"
AUDIO_OUTPUT_WAV = "prototype_audio.wav"
SEGMENT_OUTPUT_DIR = "analysis/labeled_data"

# === STAGE 0: SELECT ROSBAG INTERACTIVELY ===
def choose_bag():
    bags = [f for f in os.listdir(ROSBAG_DIR) if os.path.isdir(os.path.join(ROSBAG_DIR, f))]
    if not bags:
        raise RuntimeError("No rosbag folders found in 'rosbags/'")

    print("Available rosbag folders:")
    for i, bag in enumerate(bags):
        print(f"[{i}] {bag}")

    idx = input("Select a bag by number: ")
    try:
        idx = int(idx)
        selected = bags[idx]
    except (ValueError, IndexError):
        raise RuntimeError("Invalid selection.")

    return os.path.join(ROSBAG_DIR, selected)

# === STAGE 1: READ ROS 2 BAG FILE ===
def read_rosbag(bag_path):
    reader = SequentialReader()
    storage_opts = StorageOptions(uri=bag_path, storage_id='sqlite3')
    conv_opts = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_opts, conv_opts)
    reader.set_filter(StorageFilter(topics=[AUDIO_TOPIC, ROBOT_TOPIC]))

    audio_msgs, robot_msgs = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == AUDIO_TOPIC:
            audio_msgs.append((data, t))
        elif topic == ROBOT_TOPIC:
            robot_msgs.append((data, t))
    return audio_msgs, robot_msgs

# === STAGE 2: SAVE AUDIO TO WAV ===
def save_audio(audio_msgs, output_path):
    combined = AudioSegment.empty()
    for i, (msg_bytes, _) in enumerate(audio_msgs):
        samples = np.frombuffer(msg_bytes, dtype=np.int16)
        seg = AudioSegment(
            samples.tobytes(),
            frame_rate=44100,
            sample_width=2,
            channels=1
        )
        seg = seg.set_frame_rate(16000)
        seg = normalize(seg)
        combined += seg

    combined.export(output_path, format="wav")
    print(f"[INFO] Full audio saved to {output_path}")

# === STAGE 3: TRANSCRIBE AUDIO ===
def transcribe_audio(audio_path):
    model = whisper.load_model("base")
    result = model.transcribe(audio_path)
    return result['segments']

# === STAGE 4: GENERATE CLASSIFICATION PROMPT ===
def generate_prompt(transcript):
    return f"""
You are an intent classification assistant.

Segment the following transcript into logical utterances or sentences. For each utterance, classify it using one or more of the following intent categories.

### 📚 Intent Taxonomy (Detailed + Refined)

1. **generalization_zone**
   - High-level descriptions of **goals, intentions, or reasoning** behind the task.
   - Explains *why* something is done, or provides an overarching strategy.
   - ✅ Examples:
     - "The idea is to avoid spilling."
     - "We want to ensure the object is stable."
     - "Now we begin the task of stacking these."
   - ❌ Not for direct movement commands or conditions.

2. **constraint_zone**
   - Statements about **physical, spatial, or logical constraints** that must be respected during the task.
   - Includes descriptions of current spatial state, placement, or task requirements.
   - ✅ Examples:
     - "You are now above the fruit box."
     - "The cup should not tip over."
     - "It must stay inside the container."
   - ❌ Not for how actions are executed—use compliance categories instead.

3. **compliance_soft**
   - Actions requiring **gentle, adaptive, or low-force** execution.
   - Often involves delicate handling or motion near sensitive objects or surfaces.
   - ✅ Examples:
     - "Lower it gently."
     - "Set it down slowly."
     - "Let it touch the surface softly."
   - ❌ Not used for firm or decisive control.

4. **compliance_firm**
   - Actions requiring **forceful, secure, or confident control**.
   - Often used for gripping, lifting, or overcoming resistance.
   - ✅ Examples:
     - "Grip it tightly."
     - "Lift the box straight up."
     - "Push through until you reach the edge."
   - ❌ Not used for soft or reactive movements.

5. **repeat_or_demo**
   - Used when **repeating, paraphrasing, confirming**, or **demonstrating** prior steps.
   - Also includes verbal reinforcement, corrections, and examples.
   - ✅ Examples:
     - "As I said earlier, place it gently."
     - "Let me show you again."
     - "You're now holding the object."

---

### 🧠 Annotation Guidelines

- Label **current spatial or physical state descriptions** (e.g., "You're above the fruit box") as `constraint_zone` if they affect the task.
- Use `repeat_or_demo` when the speaker restates, confirms, or reinforces a previous instruction—even subtly.
- If multiple **short adjacent sentences form a single intent**, group and classify them together.
- Imperative or indirect instructions (e.g., "Go here", "Drop it now", "Bring it closer") often carry intent even if implicit—classify accordingly.
- Favor classification when there's **clear functional relevance**, even if the language is casual or indirect.

---

Return your output as a JSON list where each item includes:
- "text": the utterance
- "intents": a list of one or more applicable intent labels

Transcript:
\"\"\"
{transcript}
\"\"\"

Output format:
[
  {{
    "text": "...",
    "intents": ["...", "..."]
  }},
  ...
]
"""

# === STAGE 4.5: CHECK TIMESTAMP ALIGNMENT ===
def check_timestamp_alignment(audio_msgs, robot_msgs):
    def get_time_bounds(msgs):
        timestamps = [t for (_, t) in msgs]
        return min(timestamps), max(timestamps)

    audio_start, audio_end = get_time_bounds(audio_msgs)
    robot_start, robot_end = get_time_bounds(robot_msgs)

    print("\n⏱️ Timestamp Alignment:")
    print(f"Audio     : start = {audio_start}, end = {audio_end}")
    print(f"RobotState: start = {robot_start}, end = {robot_end}")

    tolerance_ns = 10_000_000  # 10 ms
    if abs(audio_start - robot_start) > tolerance_ns or abs(audio_end - robot_end) > tolerance_ns:
        print("⚠️  Timestamps may not align closely. Consider syncing or trimming.")
    else:
        print("✅ Audio and robot state timestamps are reasonably aligned.")

# === STAGE 5: MAIN WORKFLOW ===
def process_bag():
    os.makedirs(SEGMENT_OUTPUT_DIR, exist_ok=True)
    bag_path = choose_bag()
    print(f"[INFO] Using bag: {bag_path}")

    audio_msgs, robot_msgs = read_rosbag(bag_path)
    check_timestamp_alignment(audio_msgs, robot_msgs)

    print("Saving full audio…")
    save_audio(audio_msgs, AUDIO_OUTPUT_WAV)

    print("Transcribing…")
    segments = transcribe_audio(AUDIO_OUTPUT_WAV)
    transcript = " ".join(seg['text'] for seg in segments)

    prompt = generate_prompt(transcript)

    print("\n" + "="*90)
    print("🧠 COPY & PASTE THIS INTO CHATGPT:\n")
    print(prompt)
    print("="*90)

if __name__ == "__main__":
    process_bag()
