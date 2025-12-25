cmake_minimum_required(VERSION 3.16)
project(plant_sensor_can_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(utils)
add_subdirectory(src/can)
add_subdirectory(src/plant)
add_subdirectory(src/sim)
