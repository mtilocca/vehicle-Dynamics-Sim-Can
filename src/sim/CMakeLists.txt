cmake_minimum_required(VERSION 3.16)

add_executable(sim_main
    sim_main.cpp
    sim_app.cpp
    # tick_clock.hpp is header-only; no need to list
)

target_include_directories(sim_main PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(sim_main PRIVATE plant utils)
