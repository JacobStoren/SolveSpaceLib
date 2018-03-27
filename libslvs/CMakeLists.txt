cmake_minimum_required(VERSION 3.2.0 FATAL_ERROR)

# Platform utilities

if(WIN32)
	add_definitions(-D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS)
endif()

set(util_SOURCES
    platform/unixutil.cpp)

# libslvs

set(libslvs_SOURCES
    util.cpp
    entity.cpp
    expr.cpp
    constrainteq.cpp
    system.cpp
)

set(libslvs_HEADERS
	dsc.h
	expr.h
	polygon.h
	resource.h
	sketch.h
    solvespace.h
    platform/platform.h
	include/slvs.h
	srf/surface.h
)

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

