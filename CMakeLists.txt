cmake_minimum_required (VERSION 3.15 FATAL_ERROR)

project(octree)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(octree INTERFACE)
target_include_directories(octree INTERFACE include)