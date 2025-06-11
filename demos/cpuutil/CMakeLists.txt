add_executable(vt-cpuutil main.c)
target_link_libraries(vt-cpuutil vtrenderlib)

if(NOT MSVC)
    target_link_libraries(vt-cpuutil m)
endif()

target_include_directories(vt-cpuutil PRIVATE ${CMAKE_SOURCE_DIR})
