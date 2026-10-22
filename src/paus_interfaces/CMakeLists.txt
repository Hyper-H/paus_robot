cmake_minimum_required(VERSION 3.8)
project(paus_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UltrasoundFrame.msg"
  "msg/ThyroidSegmentationResult.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

ament_package()
