cmake_minimum_required(VERSION 3.8)
project(gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(custom_action_interfaces REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${gz-sim8_INCLUDE_DIRS}
  ${ros_gz_interfaces_INCLUDE_DIRS}
  include  # Add the directory where ThreadPool.h is located
)

# Plugin
add_library(${PROJECT_NAME} SHARED src/JointPlugin.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_action custom_action_interfaces gz-sim8 ros_gz_interfaces)
target_link_libraries(${PROJECT_NAME} ${gz-sim8_LIBRARIES}
rclcpp::rclcpp
rclcpp_action::rclcpp_action
)

# Install
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
