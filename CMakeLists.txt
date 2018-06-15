cmake_minimum_required(VERSION 2.8.12 )

project (SolveSpaceLib)

ADD_SUBDIRECTORY(libslvs)

add_executable(CDemo
    CDemo.c
    example_S_Curve.cpp)

target_link_libraries(CDemo
    libslvs)
