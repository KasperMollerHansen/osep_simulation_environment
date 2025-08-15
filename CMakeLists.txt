cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0074 NEW)

project(osep_simulation_environment)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
    px4_msgs
    rclcpp
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    Eigen3
)


add_executable(px4_msg_converter_node src/px4_msg_converter_node.cpp)
ament_target_dependencies(px4_msg_converter_node
  ${dependencies}
)

# If your include files are in include/
target_include_directories(px4_msg_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(px4_vel_controller src/px4_vel_controller.cpp)
ament_target_dependencies(px4_vel_controller
  ${dependencies}
)

target_include_directories(px4_vel_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the executables
install(TARGETS
  px4_msg_converter_node
  px4_vel_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


## Python testing ##
install(PROGRAMS
  src/testing/converter_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# Build tests if required

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()