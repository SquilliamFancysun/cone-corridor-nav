cmake_minimum_required(VERSION 3.5)
project(cone_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LabeledCone.msg"
  "msg/LabeledConeArray.msg"
  "msg/Centerline.msg"
  "msg/GateEvent.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
