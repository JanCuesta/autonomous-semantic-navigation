cmake_minimum_required(VERSION 3.8)
project(asn_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(controller_manager_msgs REQUIRED)


# Mission Manager executable
add_executable(
  mission_manager
  src/mission_manager.cpp
)

ament_target_dependencies(
  mission_manager

  rclcpp
  rclcpp_action

  geometry_msgs
  std_msgs
  nav_msgs
  nav2_msgs
  std_srvs

  controller_manager_msgs
  
)


# Install executable
install(
  TARGETS mission_manager
  DESTINATION lib/${PROJECT_NAME}
)


# Tests / linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
