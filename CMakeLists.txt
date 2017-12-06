cmake_minimum_required (VERSION 3.5)

project (graphics17b_hw03)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED on)
set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# compile flags
if(APPLE)
    add_definitions(-Wno-missing-braces)
endif(APPLE)
if(WIN32)
    add_definitions(/D_CRT_SECURE_NO_WARNINGS /wd4018 /wd4244 /wd4305 /wd4800 /wd4267)
endif(WIN32)

add_executable(pathtrace
    src/json.hpp
    src/stb_image.h
    src/stb_image_write.h
    src/stb_image_resize.h
    src/tinyexr.h
    src/yocto_gl.h
    src/pathtrace.cpp)

if(UNIX AND NOT APPLE)
    find_package(Threads REQUIRED)
    target_link_libraries(pathtrace Threads::Threads)
endif(UNIX AND NOT APPLE)
