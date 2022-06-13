cmake_minimum_required(VERSION 3.8)
project(ichthus_driver)

# print include directory path
# message($<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

# set(CMAKE_CXX_STANDARD 17)
set(LIB_CAN ${CMAKE_SOURCE_DIR}/lib/libcan.so)
set(LIB_DBCPPP ${CMAKE_SOURCE_DIR}/lib/libdbcppp.so)
set(LIB_XML2910 ${CMAKE_SOURCE_DIR}/lib/libxml2.so.2.9.10)
set(LIB_XML2 ${CMAKE_SOURCE_DIR}/lib/libxml2.so)
set(LIB_XMLMM ${CMAKE_SOURCE_DIR}/lib/libxmlmm.so)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ichthus_msgs REQUIRED)

include_directories(include)

# add kia_reader Shared Object File
add_library(kia_reader SHARED src/kia/kia_reader.cpp)
target_compile_features(kia_reader PRIVATE cxx_std_17)
ament_target_dependencies(kia_reader rclcpp rcl_interfaces std_msgs ichthus_msgs)
target_include_directories(kia_reader
  PUBLIC
  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
target_link_libraries(kia_reader
  dl ${LIB_CAN} ${LIB_DBCPPP} ${LIB_XML2910} ${LIB_XML2} ${LIB_XMLMM}
)

add_executable(kia_reader_node src/kia/kia_node.cpp)
target_compile_features(kia_reader_node PRIVATE cxx_std_17)
ament_target_dependencies(kia_reader_node rclcpp)
target_link_libraries(kia_reader_node kia_reader)

# add mcm_reader Shared Object File
add_library(mcm_management SHARED src/mcm/mcm_management.cpp)
target_compile_features(mcm_management PRIVATE cxx_std_17)
ament_target_dependencies(mcm_management rclcpp rcl_interfaces ichthus_msgs)
target_include_directories(mcm_management
  PUBLIC
  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
target_link_libraries(mcm_management
  dl ${LIB_CAN} ${LIB_DBCPPP} ${LIB_XML2910} ${LIB_XML2} ${LIB_XMLMM}
)

add_executable(mcm_management_node src/mcm/mcm_node.cpp)
target_compile_features(mcm_management_node PRIVATE cxx_std_17)
ament_target_dependencies(mcm_management_node rclcpp)
target_link_libraries(mcm_management_node mcm_management)

install(TARGETS
  kia_reader_node
  mcm_management_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  ${LIB_XMLMM}
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  cfg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
