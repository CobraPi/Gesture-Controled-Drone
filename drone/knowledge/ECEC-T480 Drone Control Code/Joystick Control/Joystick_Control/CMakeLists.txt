cmake_minimum_required(VERSION 3.3)
project(Joystick_Control)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    HCD/bk2421.cpp
    HCD/bk2421.h
    HCD/blocks.h
    HCD/HCD.cpp
    HCD/HCD.h
    Joystick_Control.ino)

add_executable(Joystick_Control ${SOURCE_FILES})