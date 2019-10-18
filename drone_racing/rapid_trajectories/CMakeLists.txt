cmake_minimum_required(VERSION 2.8.3)
project(rapid_trajectories)

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_add_library(${PROJECT_NAME} src/RapidTrajectoryGenerator.cpp src/SingleAxisTrajectory.cpp)

cs_export()
