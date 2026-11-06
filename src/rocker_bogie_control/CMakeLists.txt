cmake_minimum_required(VERSION 3.8)
project(rocker_bogie_control)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(phoenix6 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rocker_bogie_system.cpp
)
get_filename_component(PHOENIX6_LIBRARY_DIR "${phoenix6_DIR}/.." ABSOLUTE)
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "${PHOENIX6_LIBRARY_DIR}"
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  phoenix6
)

# Export the plugin XML
pluginlib_export_plugin_description_file(hardware_interface rocker_bogie_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(FILES rocker_bogie_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
