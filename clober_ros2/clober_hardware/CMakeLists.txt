cmake_minimum_required(VERSION 3.5)
project(clober_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roas_base REQUIRED)
find_package(clober_msgs REQUIRED)

set(
  DEPENDENCIES
  "rclcpp"
  "hardware_interface"
  "pluginlib"
  "realtime_tools"
  "std_msgs"
  "roas_base"
  "clober_msgs"
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/clober_hardware.cpp
)
target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

pluginlib_export_plugin_description_file(hardware_interface clober_hardware.xml)

install(
  TARGETS ${PROJECT_NAME} 
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_dependencies(${DEPENDENCIES})
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()