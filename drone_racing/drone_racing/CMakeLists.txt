cmake_minimum_required(VERSION 2.8.3)
project(drone_racing)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
catkin_simple()

cs_add_library(drone_racing_base src/drone_racing.cpp)
cs_add_library(depth_processing src/data_saver.cpp)
cs_add_library(visualizer src/visualizer.cpp)
cs_add_library(trajectory_manager src/trajectory_manager.cpp)
cs_add_library(global_trajectory src/global_trajectory.cpp)
cs_add_library(gate_replacer src/gate_replacer.cpp)
cs_add_library(trajectory_base src/trajectory_base.cpp)
cs_add_library(minimum_jerk_trajectory src/minimum_jerk_trajectory.cpp)
cs_add_library(gazebo_rviz_visualizer src/gazebo_rviz_visualizer.cpp)

cs_add_executable(drone_racing_node src/drone_racing_node.cpp)
target_link_libraries(drone_racing_node drone_racing_base)
target_link_libraries(drone_racing_node depth_processing)
target_link_libraries(drone_racing_node visualizer)
target_link_libraries(drone_racing_node trajectory_manager)
target_link_libraries(drone_racing_node global_trajectory)
target_link_libraries(drone_racing_node gate_replacer)
target_link_libraries(drone_racing_node trajectory_base)
target_link_libraries(drone_racing_node minimum_jerk_trajectory)
target_link_libraries(drone_racing_node gazebo_rviz_visualizer)
target_link_libraries(drone_racing_node ${OpenCV_LIBS})

cs_export()

# CMake Indexing
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
FILE(GLOB_RECURSE ProtoFiles "proto/*")
add_custom_target(protos SOURCES ${ProtoFiles})
