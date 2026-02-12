cmake_minimum_required(VERSION 3.8)
project(dataset_logger)

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
find_package(morai_msgs REQUIRED)

add_library(dataset_logger src/data_logger.cpp)

target_include_directories(dataset_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(dataset_logger ${OpenCV_LIBS})

add_executable(dataset_logger_node src/dataset_logger_node.cpp)
ament_target_dependencies(dataset_logger_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  morai_msgs
)

target_link_libraries(dataset_logger_node dataset_logger ${OpenCV_LIBS})

install(TARGETS dataset_logger dataset_logger_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(dataset_logger)
ament_export_dependencies(OpenCV)

ament_package()
