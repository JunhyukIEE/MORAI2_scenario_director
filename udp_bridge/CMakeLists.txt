cmake_minimum_required(VERSION 3.8)
project(udp_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(morai_msgs REQUIRED)

include_directories(include)

add_executable(udp_receiver_node src/udp_receiver_node.cpp)
ament_target_dependencies(udp_receiver_node
  rclcpp
  std_msgs
  nav_msgs
)

add_executable(udp_sender_node src/udp_sender_node.cpp)
ament_target_dependencies(udp_sender_node
  rclcpp
  morai_msgs
)

add_executable(camera_receiver_node src/camera_receiver_node.cpp)
ament_target_dependencies(camera_receiver_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  udp_receiver_node
  udp_sender_node
  camera_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
