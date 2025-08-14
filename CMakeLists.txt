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
find_package(rosidl_default_generators REQUIRED)
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
    builtin_interfaces
)


# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Action.msg"
  DEPENDENCIES builtin_interfaces
)




# Build px4_bridge executable
add_executable(px4_bridge src/px4_bridge.cpp)
target_include_directories(px4_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(px4_bridge ${dependencies})
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(px4_bridge ${cpp_typesupport_target} px4_offboard_control)

install(TARGETS px4_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Build tests if required

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()