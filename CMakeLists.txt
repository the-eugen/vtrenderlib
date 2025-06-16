cmake_minimum_required(VERSION 3.10)
project(vtrenderlib C)

add_library(vtrenderlib STATIC vtrenderlib.c)

target_include_directories(vtrenderlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    add_compile_definitions(_WIN32_)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_compile_definitions(_LINUX_)
endif()

add_subdirectory(demos)
