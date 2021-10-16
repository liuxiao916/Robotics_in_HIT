cmake_minimum_required(VERSION 3.10.0)
project(Scara)

find_package(Eigen3 REQUIRED)
MESSAGE( STATUS "this var key = ${EIGEN3_INCLUDE_DIR}.")
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
include_directories(./SCARA)


add_executable(Scara ./SCARA/main.cpp ./SCARA/Srobotconfig.cpp)

