cmake_minimum_required(VERSION 3.8)
project(cobot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(cobot_hardware SHARED
src/cobot_hardware.cpp)

target_include_directories(cobot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cobot_hardware
hardware_interface
pluginlib
rclcpp
rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface cobot_hardware.xml)

install(TARGETS cobot_hardware
  EXPORT cobot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include
  DESTINATION include
)
ament_package()
