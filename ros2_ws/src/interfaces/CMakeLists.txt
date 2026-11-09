cmake_minimum_required(VERSION 3.10)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# .msg / .action 파일 → C++ · Python 코드 자동 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/TaskText.msg"
  "msg/TrafficLight.msg"
  "msg/SignalRecord.msg"
  "msg/VerifyPuck.msg"
  "msg/VerifyRecord.msg"
  "msg/CargoSlot.msg"
  "msg/CargoState.msg"
  "msg/MissionStatus.msg"
  "msg/MotionMode.msg"
  "action/ArmTask.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
