cmake_minimum_required(VERSION 3.2.0 FATAL_ERROR)

project (SolveSpaceLib)

ADD_SUBDIRECTORY(libslvs)

add_executable(CDemo
    CDemo.c
    example_S_Curve.cpp
	SolveSpaceSystem.h
	SolveSpaceSystem.cpp)

target_link_libraries(CDemo
    libslvs)
