add_executable(vt-loh main.c)
target_link_libraries(vt-loh vtrenderlib)

if(NOT MSVC)
    target_link_libraries(vt-loh m)
endif()

target_include_directories(vt-loh PRIVATE ${CMAKE_SOURCE_DIR})
