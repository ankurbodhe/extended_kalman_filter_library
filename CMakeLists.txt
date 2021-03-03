project(Extended_Kalman_Filter)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++0x)

find_package(yaml-cpp PATHS <yaml-cpp-install-dir>)

set(sources
    src/main.cpp 
    src/file_parse_util.cpp
    src/ekf.cpp
    src/ekf_math_util.cpp)

include_directories(INCLUDE include )

add_executable(Extended_Kalman_Filter ${sources})

target_link_libraries(Extended_Kalman_Filter ${YAMLCPP_LIBRARIES})