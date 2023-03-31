cmake_minimum_required (VERSION 3.15 FATAL_ERROR)

project(Octree)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(octree INTERFACE)
target_include_directories(octree INTERFACE include)

add_executable(tests src/tests.cpp)
target_link_libraries(tests PUBLIC octree)

add_executable(tests_alt src/tests_alt.cpp)
target_link_libraries(tests_alt PUBLIC octree)

add_executable(example src/example.cpp)
target_link_libraries(example PUBLIC octree)