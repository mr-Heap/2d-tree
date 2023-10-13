cmake_minimum_required(VERSION 3.23)
project(2d_tree)

set(CMAKE_CXX_STANDARD 17)

add_executable(2d_tree src/2dtree.cpp src/main.cpp include/primitives.h)
