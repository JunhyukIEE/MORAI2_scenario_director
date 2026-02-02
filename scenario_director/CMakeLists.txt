cmake_minimum_required(VERSION 3.8)
project(scenario_director)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_library(scenario_core
  src/line_manager.cpp
  src/overtake_strategy.cpp
  src/pure_pursuit.cpp
  src/slow_in_out.cpp
  src/track_boundary.cpp
  src/waypoint_selector.cpp
)

ament_target_dependencies(scenario_core
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  ament_index_cpp
)

target_link_libraries(scenario_core
  yaml-cpp
  ${OpenCV_LIBS}
)

add_executable(scenario_director_node src/scenario_director_node.cpp)
ament_target_dependencies(scenario_director_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  ament_index_cpp
)

target_link_libraries(scenario_director_node scenario_core ${OpenCV_LIBS} yaml-cpp)

add_executable(npc_controller_node src/npc_controller_node.cpp)
ament_target_dependencies(npc_controller_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(npc_controller_node scenario_core ${OpenCV_LIBS} yaml-cpp)

install(TARGETS
  scenario_core
  scenario_director_node
  npc_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
