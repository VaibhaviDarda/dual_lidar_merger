cmake_minimum_required(VERSION 3.8)
project(dual_lidar_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# ── Merger Node ───────────────────────────────────────────────────────────────
add_executable(dual_lidar_merger_node
  src/dual_lidar_merger_node.cpp
)

target_include_directories(dual_lidar_merger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(dual_lidar_merger_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  laser_geometry
  pcl_conversions
)

target_link_libraries(dual_lidar_merger_node
  ${PCL_LIBRARIES}
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS dual_lidar_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
