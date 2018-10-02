cmake_minimum_required(VERSION 2.8.3)
project(example_action_server)

find_package(catkin_simple REQUIRED)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# Executables
cs_add_executable(client_action_server src/client_action_server.cpp)
cs_add_executable(P3_action_server src/P3_action_server.cpp)
cs_install()
cs_export()
    
