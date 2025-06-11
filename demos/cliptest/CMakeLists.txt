add_executable(vt-cliptest main.c)
target_link_libraries(vt-cliptest vtrenderlib)

if(NOT MSVC)
    target_link_libraries(vt-cliptest m)
endif()

target_include_directories(vt-cliptest PRIVATE ${CMAKE_SOURCE_DIR})
