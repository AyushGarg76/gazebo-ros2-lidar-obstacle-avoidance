cmake_minimum_required(VERSION 3.16)
project(lidar_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

add_executable(lidar_node lidar_node.cpp)

target_link_libraries(lidar_node
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
)