cmake_minimum_required(VERSION 3.5)
project(flatros2_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(flatros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BenchmarkData.msg"
  DEPENDENCIES std_msgs
)

set(own_msgs_TARGETS)
foreach(typesupport_name rosidl_typesupport_cpp rosidl_typesupport_introspection_cpp)
  rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} ${typesupport_name})
  list(APPEND own_msgs_TARGETS ${typesupport_target})
endforeach()

add_executable(benchmark_sub src/benchmark_sub.cpp)
target_link_libraries(benchmark_sub 
 flatros2::flatros2 rclcpp::rclcpp
 ${std_msgs_TARGETS} ${own_msgs_TARGETS}
)

install(
  TARGETS benchmark_sub
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS 
    scripts/analyze_latency.py
    scripts/benchmark_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
