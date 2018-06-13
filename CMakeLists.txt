
ADD_SUBDIRECTORY(libslvs)

add_executable(CDemo
    CDemo.c
    WellPathSCreator.cpp)

target_link_libraries(CDemo
    slvs)
