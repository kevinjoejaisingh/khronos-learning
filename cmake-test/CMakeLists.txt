cmake_minimum_required(VERSION 3.10)
project(SensorTest)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(sensors sensor.cpp depth_sensor.cpp)
add_executable(sensor_test main.cpp)
target_link_libraries(sensor_test sensors)