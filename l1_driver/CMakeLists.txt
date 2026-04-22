cmake_minimum_required(VERSION 3.8)
project(l1_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(l1_driver_node src/l1_driver_node.cpp src/l1_parser.cpp)

target_include_directories(l1_driver_node PRIVATE include)

ament_target_dependencies(l1_driver_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS l1_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
