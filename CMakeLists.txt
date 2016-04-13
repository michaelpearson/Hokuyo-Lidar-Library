cmake_minimum_required(VERSION 3.3)
project(Lidar_Library)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -std=c99")

set(SOURCE_FILES main.c)
add_executable(Lidar_Library ${SOURCE_FILES})