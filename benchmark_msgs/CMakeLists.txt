cmake_minimum_required(VERSION 3.5)
project(benchmark_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(std_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BenchmarkData.msg"
  "msg/Stamped10B.msg"
  "msg/Stamped100KB.msg"
  "msg/Stamped1MB.msg"
  "msg/Stamped2MB.msg"
  "msg/Stamped4MB.msg"
  DEPENDENCIES std_msgs
)

# Export necessary dependencies
ament_export_dependencies(
  rosidl_default_runtime
  rosidl_typesupport_c
  rosidl_typesupport_cpp
  std_msgs
)

ament_package()
