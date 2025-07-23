# Audio Recording and Processing with ROS 2

This repository contains ROS 2 nodes for recording audio, publishing it over ROS topics, saving to WAV files, and synchronizing with image data (another topic of interest) in rosbags.

## Features

- 🎤 **Audio Recording**: Capture audio from microphone using `sounddevice`
- 🔁 **ROS 2 Integration**: Publish audio data as `UInt8MultiArray` messages
- 💾 **File Saving**: Save recordings as WAV files with timestamps
- 🖼️ **Synchronization**: Record audio in sync with image topics
- 📦 **Rosbag Storage**: Store synchronized audio and image data in rosbags

---

## Installation

### Prerequisites

- ROS 2 Humble (or compatible distribution)
- Python 3.8+
- Audio input device (microphone)

### Dependencies

```bash
sudo apt-get install portaudio19-dev libsndfile1 ffmpeg
pip install sounddevice soundfile numpy pydub
````

### Building the Package

```bash
cd ~/your_ros2_ws/src
git clone <this-repo>
cd ..
colcon build --symlink-install
source install/setup.bash
```

---

## Nodes

### 🎙️ Audio Recorder (`record2.py`)

Records audio and publishes it to ROS topics while saving it to WAV and rosbag files.

**Usage:**

```bash
ros2 run audio_recorder record2
```

**Key Features:**

* Records continuously until Enter is pressed
* Saves `.wav` files to `~/audio_recordings/`
* Saves rosbag to timestamped folder `./audio_image_bag_<timestamp>/`
* Publishes to `/audio_data`
* Syncs audio with `/image` topic

### 🔊 Audio Player (`play2.py`)

Subscribes to `/audio_data` and plays audio in real-time.

**Usage:**

```bash
ros2 run audio_recorder play2
```

---

## Configuration

### 🔧 Audio Parameters

Edit in `record2.py`:

* `fs`: Sample rate (default: `16000`)
* `channels`: Mono (1) or stereo (2)
* `dtype`: Audio format (default: `'int16'`)

### 📁 File Storage

* **WAV Files:** `~/audio_recordings/`
* **Rosbags:** `./audio_image_bag_<timestamp>/`

### 📡 QoS Settings

All QoS parameters are defined centrally in:

```python
audio_recorder/utils.py
```

#### Example:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

def get_audio_qos():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        depth=10
    )
```

Use `get_audio_qos()` in both `record2.py` and `play2.py` for consistent topic behavior.

---

## 📷 Synchronized Recording

To record audio in sync with a camera/image publisher:

1. Start your camera node (e.g. publishing to `/image`)
2. Start the audio recorder:

   ```bash
   ros2 run audio_recorder record2
   ```
3. Press `Enter` to stop recording

The rosbag will contain synchronized messages from `/audio_data` and `/image`.

---

## ▶️ Playing Back Recordings

### From Rosbag:

```bash
ros2 bag play audio_image_bag_<timestamp>
```

### From WAV:

Use any standard audio player to listen to files in `~/audio_recordings/`.

---

## 🛠 Troubleshooting

| Issue                 | Solution                                          |
| --------------------- | ------------------------------------------------- |
| `Permission denied`   | Make sure you have microphone access              |
| `ModuleNotFoundError` | Install missing Python dependencies               |
| Rosbag errors         | Ensure the directory is removed or uniquely named |
| `QoS incompatible`    | Use the same `QoSProfile` in all nodes            |

---

## 📄 License

Licensed under the [Apache License 2.0](LICENSE).

---

## 🙌 Acknowledgements

* `sounddevice` for audio I/O
* `soundfile` for WAV file writing
* ROS 2 for real-time communication

---
