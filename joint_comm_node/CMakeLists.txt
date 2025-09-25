cmake_minimum_required(VERSION 3.8)
project(joint_comm_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial_driver REQUIRED)

message(STATUS "serial_driver_INCLUDE_DIRS = ${serial_driver_INCLUDE_DIRS}")
message(STATUS "serial_driver_LIBRARIES = ${serial_driver_LIBRARIES}")

add_executable(joint_comm_node
  src/joint_comm_node.cpp
)

ament_target_dependencies(joint_comm_node
  rclcpp
  sensor_msgs
  serial_driver
)

install(TARGETS
  joint_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

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
