cmake_minimum_required(VERSION 3.2.0 FATAL_ERROR)

# platform utilities

if(WIN32)
    set(util_SOURCES
        platform/w32util.cpp
)
	add_definitions(-D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS)
else()
    set(util_SOURCES
        platform/unixutil.cpp)
endif()

if(APPLE)
    set(util_LIBRARIES
        ${APPKIT_LIBRARY})
endif()

# libslvs

set(libslvs_SOURCES
    util.cpp
    entity.cpp
    expr.cpp
    constraint.cpp
    constrainteq.cpp
    system.cpp
    platform/platform.cpp
)

set(libslvs_HEADERS
    solvespace.h
    platform/platform.h)

add_library(slvs SHARED
    ${libslvs_SOURCES}
    ${libslvs_HEADERS}
    ${util_SOURCES}
    lib.cpp)

target_compile_definitions(slvs
    PRIVATE -DLIBRARY)

target_include_directories(slvs
    PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include 
    ${CMAKE_CURRENT_LIST_DIR} )

target_link_libraries(slvs
    ${util_LIBRARIES})

set_target_properties(slvs PROPERTIES
    PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/include/slvs.h
    VERSION ${solvespace_VERSION_MAJOR}.${solvespace_VERSION_MINOR}
    SOVERSION 1)

