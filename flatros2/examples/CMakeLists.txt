find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(benchmark_msgs REQUIRED)

add_executable(edge_detector_node edge_detector_node.cpp)
target_link_libraries(edge_detector_node PRIVATE
    ${PROJECT_NAME} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
    cv_bridge::cv_bridge opencv_core opencv_objdetect rclcpp::rclcpp)

install(
  TARGETS edge_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS
    image_capture_node.py
    image_viewer_node.py
    benchmark_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(benchmark_sub benchmark_sub.cpp)
target_link_libraries(benchmark_sub
  ${PROJECT_NAME}
)
ament_target_dependencies(benchmark_sub
  rclcpp
  benchmark_msgs
)

target_include_directories(benchmark_sub PUBLIC
  $<BUILD_INTERFACE:${benchmark_msgs_DIR}/../../../include>
)

install(
  TARGETS benchmark_sub
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(benchmark_pub_fixed_size benchmark_pub_fixed_size.cpp)
target_link_libraries(benchmark_pub_fixed_size
  ${PROJECT_NAME}
)
ament_target_dependencies(benchmark_pub_fixed_size
  rclcpp
  benchmark_msgs
)
target_include_directories(benchmark_pub_fixed_size PUBLIC
  $<BUILD_INTERFACE:${benchmark_msgs_DIR}/../../../include>
)
install(
  TARGETS benchmark_pub_fixed_size
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(benchmark_sub_fixed_size benchmark_sub_fixed_size.cpp)
target_link_libraries(benchmark_sub_fixed_size
  ${PROJECT_NAME}
)
ament_target_dependencies(benchmark_sub_fixed_size
  rclcpp
  benchmark_msgs
)
target_include_directories(benchmark_sub_fixed_size PUBLIC
  $<BUILD_INTERFACE:${benchmark_msgs_DIR}/../../../include>
)
install(
  TARGETS benchmark_sub_fixed_size
  DESTINATION lib/${PROJECT_NAME}
)
