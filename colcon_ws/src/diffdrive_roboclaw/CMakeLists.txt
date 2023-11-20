cmake_minimum_required(VERSION 3.16)
project(diffdrive_roboclaw LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  roboclaw_serial_driver
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  diffdrive_roboclaw
  SHARED
  hardware/diffbot_system.cpp
)

target_compile_features(diffdrive_roboclaw PUBLIC cxx_std_17)
target_include_directories(diffdrive_roboclaw PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/diffdrive_roboclaw>
)
ament_target_dependencies(
  diffdrive_roboclaw PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFFDRIVE_ROBOCLAW_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diffdrive_roboclaw.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/diffdrive_roboclaw
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/diffdrive_roboclaw
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/diffdrive_roboclaw
)
install(TARGETS diffdrive_roboclaw
  EXPORT export_diffdrive_roboclaw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_diffdrive_roboclaw HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
