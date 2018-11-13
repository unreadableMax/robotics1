# --------------------
# REQUIRED CMAKE SETUP
# --------------------

# set the minimum required cmake version for this project
cmake_minimum_required(VERSION 2.8.9)

# project name
project (hello_eigen)

# ------------
# DEPENDENCIES
# ------------

# search for the eigen3 package
find_package(Eigen3 REQUIRED)

# include the eigen header directories, such that #include <Eigen/...> 
# works in your C++ files
include_directories(${EIGEN3_INCLUDE_DIR})

# ---------------------
# BUILD THE ACTUAL CODE
# ---------------------

# add an executable to the project
# usage: add_executable(EXECUTABLE_NAME CPP_FILE1 CPP_FILE2 CPP_FILE3 ...)
add_executable(hello_eigen main.cpp)

# link libraries to your executable
# usage: target_link_libraries(EXECUTABLE_NAME LIBRARY1 LIBRARY2 ...)
# note : this is completely useless here, as Eigen is header-only 
target_link_libraries(hello_eigen ${EIGEN3_LIBRARIES})
