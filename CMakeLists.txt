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
)

# Include directories for your package
include_directories(include)

# Add executable for src/setpoints.cpp
add_executable(setpoints_node src/setpoints.cpp)
ament_target_dependencies(setpoints_node
  rclcpp 
  px4_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(TARGETS 
  setpoints_node
  cost_map_node
  planner_node
  vel_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Build tests if required
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

###########
# Python ##
###########

# Install the launch directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/examples/test_flight.py
  src/examples/test_flight_2.py
  src/examples/test_flight_3.py
  src/examples/test_flight_2_180.py
  src/examples/path_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install python launch files.
# install(FILES
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
