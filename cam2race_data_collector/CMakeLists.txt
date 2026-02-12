cmake_minimum_required(VERSION 3.5)
project(cam2race_data_collector)

# Default to C++14 or later
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(morai_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Executable
add_executable(data_collector_node src/data_collector_node.cpp)
ament_target_dependencies(data_collector_node
  rclcpp
  std_msgs
  sensor_msgs
  morai_msgs
  cv_bridge
  image_transport
  message_filters
  OpenCV
)

# Install
install(TARGETS
  data_collector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
