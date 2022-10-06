cmake_minimum_required(VERSION 3.5)
project(husky_gps_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED )
find_package(geographic_msgs REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIR})

set(dependencies
rclcpp
rclcpp_action
rclcpp_lifecycle
nav2_util
nav2_lifecycle_manager
nav_msgs
nav2_msgs
nav2_core
nav2_waypoint_follower
tf2_ros
tf2_geometry_msgs
robot_localization
geographic_msgs
)

# NAV2 wp client

set(nav2_wp_cl nav2_wp_follower_client)
add_executable(${nav2_wp_cl} src/${nav2_wp_cl}.cpp)
ament_target_dependencies(${nav2_wp_cl} ${dependencies})

# Gps collector

set(gps_wp_coll gps_waypoint_collector)
add_executable(${gps_wp_coll} src/${gps_wp_coll}.cpp)
ament_target_dependencies(${gps_wp_coll} ${dependencies})
target_link_libraries(${gps_wp_coll} ${YAML_CPP_LIBRARIES})



install(
  TARGETS ${gps_wp_coll}
          ${nav2_wp_cl} 
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(
  DIRECTORY launch params 
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
