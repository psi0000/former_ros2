cmake_minimum_required(VERSION 3.5)
project(clober_base)

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
find_package(clober_msgs REQUIRED)

set(
  DEPENDENCIES
  "rclcpp"
  "controller_manager"
  "roas_base"
  "clober_msgs"
)

add_executable(
  clober_base_node 
  src/clober_base.cpp
)
target_include_directories(clober_base_node PRIVATE include)
ament_target_dependencies(clober_base_node ${DEPENDENCIES})

add_executable(
  clober_gpio_node 
  src/clober_gpio.cpp
)
target_include_directories(clober_gpio_node PRIVATE include)
ament_target_dependencies(clober_gpio_node ${DEPENDENCIES})

install(
  DIRECTORY config launch lib rules
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS clober_base_node clober_gpio_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()