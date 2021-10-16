cmake_minimum_required(VERSION 3.10.0)
project(Scara)

find_package(Eigen3 REQUIRED)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
include_directories(..)
include_directories(.)
include_directories(/home/liuxiao/code/SCARA)

add_executable(Scara ./main.cpp)
