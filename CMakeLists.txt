cmake_minimum_required(VERSION 3.10)
project(vtrenderlib C)

add_library(vtrenderlib STATIC vtrenderlib.c)

target_compile_options(vtrenderlib PRIVATE -Wall -Wextra -Werror)

# Public include directory for consumers
target_include_directories(vtrenderlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(demos)
