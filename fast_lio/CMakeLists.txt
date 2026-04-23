cmake_minimum_required(VERSION 3.14)
project(fast_lio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3 -march=native)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(Eigen3 REQUIRED)

set(DEPS
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
  tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions
)

add_executable(fast_lio_node
  src/laserMapping.cpp
  src/preprocess.cpp
  src/ikd_Tree.cpp
)

target_include_directories(fast_lio_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(fast_lio_node
  ${PCL_LIBRARIES}
  pcl_common
  pcl_filters
  pcl_io
  pcl_kdtree
)
ament_target_dependencies(fast_lio_node ${DEPS})

install(TARGETS fast_lio_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
