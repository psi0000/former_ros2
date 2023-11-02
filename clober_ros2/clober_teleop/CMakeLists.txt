cmake_minimum_required(VERSION 3.5)
project(clober_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(joy REQUIRED)

set(
  DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "joy"
)

include_directories(include)

add_executable(
  clober_teleop_node
  src/teleop_joy.cpp
)
target_link_libraries(clober_teleop_node)
ament_target_dependencies(clober_teleop_node ${DEPENDENCIES})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS clober_teleop_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_package()