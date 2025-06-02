cmake_minimum_required(VERSION 3.20)
project(flatros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

pybind11_add_module(${PROJECT_NAME}_pybindings
  src/pybind11/module.cpp
  src/pybind11/destroyable.cpp
  src/pybind11/publisher.cpp
  src/pybind11/subscription.cpp
  src/pybind11/typesupport.cpp
  src/pybind11/utilities.cpp
)
target_link_libraries(${PROJECT_NAME}_pybindings PRIVATE
  ${PROJECT_NAME} rmw::rmw rcl::rcl rosidl_runtime_c::rosidl_runtime_c)
target_include_directories(${PROJECT_NAME}_pybindings PRIVATE src/pybind11/)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_pybindings
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ${PROJECT_NAME}_pybindings
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_package()
