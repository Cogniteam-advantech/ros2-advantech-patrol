cmake_minimum_required(VERSION 3.8)
project(advantech_patrol)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Find dependencies

# Include directories
include_directories(include)

# Create library for PatrolManager and MoveBaseController
add_library(${PROJECT_NAME}_lib
  # src/PatrolManager.cpp
  #src/MoveBaseController.cpp
  src/LogManager.cpp
)

# Specify dependencies for the library
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  cv_bridge
  OpenCV
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_lib ${OpenCV_LIBRARIES})

# Create executable
add_executable(${PROJECT_NAME}_node
  src/advantech_patrol_node.cpp
)

# Specify dependencies for the executable
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
  FILES_MATCHING PATTERN "*.launch.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
  FILES_MATCHING PATTERN "*.yaml"
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  cv_bridge
  OpenCV
)

ament_package()