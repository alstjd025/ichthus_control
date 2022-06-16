cmake_minimum_required(VERSION 3.8)
project(pid_control)

# print include directory path
# message($<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
# message($<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tier4_autoware_utils REQUIRED)
find_package(ichthus_msgs REQUIRED)

include_directories(include)

# add pid_controller Static Object File
add_library(pid_controller STATIC src/pid_controller.cpp)
target_compile_features(pid_controller PRIVATE cxx_std_17)
ament_target_dependencies(pid_controller rclcpp rcl_interfaces
  ichthus_msgs std_msgs geometry_msgs tier4_autoware_utils sensor_msgs)
target_include_directories(pid_controller
  PUBLIC
  #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #$<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

add_executable(pid_node src/pid_node.cpp)
ament_target_dependencies(pid_node rclcpp)
target_link_libraries(pid_node pid_controller)

install(TARGETS
  pid_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  cfg
  DESTINATION share/${PROJECT_NAME})
ament_package()
