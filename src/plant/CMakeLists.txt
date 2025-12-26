cmake_minimum_required(VERSION 3.16)

add_library(plant STATIC
    plant_model.cpp
    steer_plant.cpp
    drive_plant.cpp
    vehicle_bicycle_ackermann.cpp
    battery_plant.cpp  
    subsystem_manager.cpp        # NEW
    battery_subsystem.cpp         # NEW
    drive_subsystem.cpp           # NEW
    steer_subsystem.cpp           # NEW
)

target_include_directories(plant PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(plant PUBLIC utils)
