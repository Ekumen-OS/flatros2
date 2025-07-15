find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(face_detector_node face_detector_node.cpp)
target_link_libraries(face_detector_node PRIVATE 
    ${PROJECT_NAME} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} 
    cv_bridge::cv_bridge opencv_core opencv_objdetect rclcpp::rclcpp)

install(
  TARGETS face_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS image_capture_node.py image_viewer_node.py
  DESTINATION lib/${PROJECT_NAME})