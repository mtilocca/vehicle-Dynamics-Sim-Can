cmake_minimum_required(VERSION 3.16)
project(can_tools LANGUAGES CXX)

add_executable(vcan_random_sender vcan_random_sender.cpp)
add_executable(vcan_listener vcan_listener.cpp)

target_link_libraries(vcan_random_sender PRIVATE utils)
target_link_libraries(vcan_listener PRIVATE utils)