cmake_minimum_required(VERSION 3.5)
project(benchmark_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BenchmarkData.msg"
  DEPENDENCIES std_msgs
  # GENERATE_TYPESUPPORT "rosidl_typesupport_c"
)

ament_export_dependencies(rosidl_default_runtime std_msgs)
ament_package()
