cmake_minimum_required(VERSION 3.5)
project(former_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(roas_base REQUIRED)
find_package(former_msgs REQUIRED)

set(
  DEPENDENCIES
  "rclcpp"
  "controller_manager"
  "roas_base"
  "former_msgs"
)

add_executable(
  former_base_node 
  src/former_base.cpp
)
target_include_directories(former_base_node PRIVATE include)
ament_target_dependencies(former_base_node ${DEPENDENCIES})

add_executable(
  former_gpio_node 
  src/former_gpio.cpp
)
target_include_directories(former_gpio_node PRIVATE include)
ament_target_dependencies(former_gpio_node ${DEPENDENCIES})

install(
  DIRECTORY config launch lib rules
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS former_base_node former_gpio_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()