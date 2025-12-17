cmake_minimum_required(VERSION 3.16)
project(can_tools LANGUAGES CXX)

add_executable(vcan_random_sender
    vcan_random_sender.cpp
    can_map.cpp
    can_codec.cpp
    tx_scheduler.cpp
    socketcan_iface.cpp

)

add_executable(vcan_listener
    vcan_listener.cpp
    can_map.cpp
    can_codec.cpp
    tx_scheduler.cpp
    socketcan_iface.cpp
)

# Make "utils/xxx.hpp" resolvable (and also lets you include from "src/..." if you want)
target_include_directories(vcan_random_sender PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

target_include_directories(vcan_listener PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(vcan_random_sender PRIVATE utils)
target_link_libraries(vcan_listener PRIVATE utils)
