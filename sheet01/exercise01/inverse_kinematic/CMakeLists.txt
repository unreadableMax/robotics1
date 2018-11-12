cmake_minimum_required(VERSION 2.8.9)
project (inverse_kinematic)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(invserse_kinematic main.cpp)
