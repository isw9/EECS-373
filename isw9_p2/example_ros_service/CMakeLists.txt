cmake_minimum_required(VERSION 2.8.3)
project(example_ros_service)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(ros_service_client_p2 src/ros_service_client_p2.cpp)
cs_add_executable(ros_service_p2 src/ros_service_p2.cpp)
cs_install()
cs_export()
