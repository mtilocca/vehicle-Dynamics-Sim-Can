cmake_minimum_required(VERSION 3.16)

# ============================================================================
# Simulation Executable
# ============================================================================

set(SIM_SOURCES
    sim_main.cpp            # Entry point and CLI argument parsing
    sim_app.cpp             # Main simulation loop (CAN closed-loop + InfluxDB)
    plant_state_packer.cpp  # PlantState → CAN frame conversion (visitor pattern)
)

add_executable(sim_main ${SIM_SOURCES})

target_include_directories(sim_main PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

# Dependency graph:
#   sim_main → plant (vehicle dynamics subsystems)
#            → can   (CAN TX/RX, signal codec)
#            → config (YAML vehicle loading)
#            → utils (logging, InfluxDB client via libcurl)
target_link_libraries(sim_main PRIVATE
    plant
    can
    config
    utils
)

target_compile_features(sim_main PRIVATE cxx_std_17)

install(TARGETS sim_main RUNTIME DESTINATION bin)

message(STATUS "sim_main configured (CAN closed-loop, InfluxDB optional)")
