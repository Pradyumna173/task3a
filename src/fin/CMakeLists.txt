cmake_minimum_required(VERSION 3.8)
project(fin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ebot_docking REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(usb_servo REQUIRED)
find_package(nav2_msgs REQUIRED)
# find_package(usb_servo REQUIRED)

add_executable(arm src/arm.cpp)
ament_target_dependencies(arm rclcpp)

add_executable(dock src/dock.cpp)
ament_target_dependencies(
  dock
  rclcpp
  geometry_msgs
  ebot_docking
  std_msgs
  sensor_msgs
  usb_servo)

add_executable(nav src/nav.cpp)
ament_target_dependencies(
  nav
  rclcpp
  nav2_msgs
  rclcpp_action
  geometry_msgs
  ebot_docking
  std_msgs)

install(TARGETS arm dock nav DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS src/arm.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
