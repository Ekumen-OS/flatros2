find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(edge_detector_node edge_detector_node.cpp)
target_link_libraries(edge_detector_node PRIVATE
    ${PROJECT_NAME} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
    cv_bridge::cv_bridge opencv_core opencv_objdetect rclcpp::rclcpp)

add_executable(image_recorder_node image_recorder_node.cpp)
target_link_libraries(image_recorder_node PRIVATE
    ${PROJECT_NAME} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
    rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp)

install(
  TARGETS 
    edge_detector_node 
    image_recorder_node
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS
    image_capture_node.py
    image_viewer_node.py
  DESTINATION lib/${PROJECT_NAME}
)
