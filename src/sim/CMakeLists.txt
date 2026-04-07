cmake_minimum_required(VERSION 3.16)

# ============================================================================
# Simulation Executable
# ============================================================================
# Orchestrates:
#   - Plant model stepping (Heavy-Duty Electric Vehicle, hardcoded params)
#   - CAN TX/RX interfaces (closed-loop control only)
#   - Sensor simulation
#   - InfluxDB telemetry logging
# ============================================================================

set(SIM_SOURCES
    sim_main.cpp                # Entry point and CLI argument parsing
    sim_app.cpp                 # Main simulation loop and orchestration
    plant_state_packer.cpp      # PlantState -> CAN frame conversion
)

add_executable(sim_main ${SIM_SOURCES})

target_include_directories(sim_main PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

# ============================================================================
# Link Libraries
# ============================================================================
# Dependency graph:
#   sim_main -> plant -> utils (includes libcurl for InfluxDB)
#   sim_main -> can -> utils (CAN communication layer)
#   sim_main -> config (hardcoded HDV vehicle params)
# ============================================================================

target_link_libraries(sim_main PRIVATE
    plant
    can
    config
    utils
)

target_compile_features(sim_main PRIVATE cxx_std_17)

install(TARGETS sim_main
    RUNTIME DESTINATION bin
)

message(STATUS "Simulation executable configured:")
message(STATUS "  - Target: sim_main")
message(STATUS "  - Dependencies: plant, can, config, utils")
message(STATUS "  - No Lua, No yaml-cpp")
