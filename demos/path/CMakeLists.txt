add_executable(vt-path main.c)
target_link_libraries(vt-path vtrenderlib)

if(NOT MSVC)
    target_link_libraries(vt-path m)
endif()

target_include_directories(vt-path PRIVATE ${CMAKE_SOURCE_DIR})
