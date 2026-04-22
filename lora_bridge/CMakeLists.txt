cmake_minimum_required(VERSION 3.8)
project(lora_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(lora_bridge_node src/lora_bridge_node.cpp)

target_include_directories(lora_bridge_node PRIVATE include)

ament_target_dependencies(lora_bridge_node
    rclcpp std_msgs nav_msgs)

install(TARGETS lora_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
