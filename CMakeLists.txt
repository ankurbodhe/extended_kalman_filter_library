project(Extended_Kalman_Filter)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/main.cpp 
    src/file_parse_util.cpp)

include_directories(INCLUDE include)

add_executable(Extended_Kalman_Filter ${sources})
