add_executable(vt-boids main.c)
target_link_libraries(vt-boids vtrenderlib)

# math library on non-MSVC compilers
if(NOT MSVC)
    target_link_libraries(vt-boids m)
endif()

target_include_directories(vt-boids PRIVATE ${CMAKE_SOURCE_DIR})
