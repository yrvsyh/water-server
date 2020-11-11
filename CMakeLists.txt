cmake_minimum_required(VERSION 3.5)

project(WaterServer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#add_compile_options("-fsanitize=address")
#link_libraries("-fsanitize=address")

#add_definitions(-DPI)
include_directories(include)
link_directories(lib)

add_executable(WaterServer main.cpp slider.cpp slider.hpp sensor.cpp sensor.hpp serialport.cpp serialport.hpp camera.cpp camera.hpp config.cpp config.hpp waterserver.cpp waterserver.hpp)
target_link_libraries(WaterServer pthread muduo_net muduo_base avcodec avformat avutil avdevice swscale)
