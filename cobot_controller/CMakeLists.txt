cmake_minimum_required(VERSION 3.8)
project(cobot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_library(cobot_controller SHARED
  src/cobot_controller.cpp
)

target_include_directories(cobot_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS cobot_controller
  EXPORT export_cobot_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



target_link_libraries(cobot_controller controller_interface::controller_interface)

pluginlib_export_plugin_description_file(controller_interface cobot_controller.xml)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

# Export the targets
ament_export_targets(export_cobot_controller HAS_LIBRARY_TARGET)
ament_package()
