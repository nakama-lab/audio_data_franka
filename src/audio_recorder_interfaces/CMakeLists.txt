cmake_minimum_required(VERSION 3.8)
project(audio_recorder_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)  # ✅ This is required for rosbag2
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioData.msg"
  "srv/StartRecording.srv"
  "srv/PlayAudio.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
