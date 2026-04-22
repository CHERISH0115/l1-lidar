cmake_minimum_required(VERSION 3.8)
project(qt_visualizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)

add_executable(qt_visualizer_node
    src/main.cpp
    src/main_window.cpp
    src/cloud_widget.cpp
    src/ros_worker.cpp
    include/main_window.hpp
    include/cloud_widget.hpp
    include/ros_worker.hpp
)

target_include_directories(qt_visualizer_node PRIVATE include)

ament_target_dependencies(qt_visualizer_node
    rclcpp sensor_msgs nav_msgs std_msgs tf2)

target_link_libraries(qt_visualizer_node
    Qt5::Widgets Qt5::OpenGL)

install(TARGETS qt_visualizer_node DESTINATION lib/${PROJECT_NAME})

ament_package()
