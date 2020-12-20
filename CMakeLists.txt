cmake_minimum_required(VERSION 3.5)

project(WaterServer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#add_compile_options("-fsanitize=address")
#link_libraries("-fsanitize=address")

add_definitions(-DPI)

#add_definitions(-DSPDLOG_FMT_EXTERNAL)
#add_definitions(-DSPDLOG_COMPILED_LIB)

add_executable(WaterServer main.cpp serialport.cpp)
target_link_libraries(WaterServer event pthread wiringPi)
