cmake_minimum_required(VERSION 3.8)
project(sar_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriverHealth.msg"
  "msg/FSMEvent.msg"
  "msg/MissionCoverage.msg"
  "msg/UAVState.msg"
  "msg/DetectionEvent.msg"
  "msg/Alert.msg"
  "srv/GetOccupancyGrid.srv"
  DEPENDENCIES std_msgs
)

ament_package()
