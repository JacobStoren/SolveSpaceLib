
ADD_SUBDIRECTORY(libslvs)

add_executable(CDemo
    CDemo.c)

target_link_libraries(CDemo
    slvs)
