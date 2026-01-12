cmake_minimum_required(VERSION 3.16)

# ============================================================================
# Simulation Executable - Main Application
# ============================================================================
# The sim_main executable orchestrates:
#   - Lua scenario runtime
#   - Plant model stepping
#   - CAN TX/RX interfaces
#   - Sensor simulation
#   - InfluxDB telemetry logging
# ============================================================================

# ----------------------------------------------------------------------------
# Core simulation sources
# ----------------------------------------------------------------------------

set(SIM_SOURCES
    sim_main.cpp                # Entry point and CLI argument parsing
    sim_app.cpp                 # Main simulation loop and orchestration
    lua_runtime.cpp             # Lua scenario interpreter
    plant_state_packer.cpp      # PlantState → CAN frame conversion
)

# ----------------------------------------------------------------------------
# CAN interface sources (temporary - TODO: create can library)
# ----------------------------------------------------------------------------
# These will eventually move to a separate can library, but for now
# we include them directly to avoid circular dependencies during development
# ----------------------------------------------------------------------------

set(CAN_SOURCES
    ${CMAKE_SOURCE_DIR}/src/can/socketcan_iface.cpp      # Linux SocketCAN interface
    ${CMAKE_SOURCE_DIR}/src/can/can_map.cpp              # CAN signal database
    ${CMAKE_SOURCE_DIR}/src/can/can_codec.cpp            # Signal encoding/decoding
    ${CMAKE_SOURCE_DIR}/src/can/tx_scheduler.cpp         # Multi-rate frame transmission
    ${CMAKE_SOURCE_DIR}/src/can/actuator_cmd_decoder.cpp # CAN RX command parsing
)

# ----------------------------------------------------------------------------
# Create simulation executable
# ----------------------------------------------------------------------------

add_executable(sim_main ${SIM_SOURCES} ${CAN_SOURCES})

# ----------------------------------------------------------------------------
# Include directories
# ----------------------------------------------------------------------------

target_include_directories(sim_main PRIVATE
    ${CMAKE_SOURCE_DIR}           # Root for absolute includes
    ${CMAKE_SOURCE_DIR}/src       # Source root for relative includes
)

# ============================================================================
# Lua Dependency - Cross-platform configuration
# ============================================================================
# Windows (vcpkg): Use CMake's FindLua
# Unix: Prefer pkg-config for lua5.4/lua5.3
# ============================================================================

if(WIN32)
    # Windows: Use find_package (vcpkg provides LUA_INCLUDE_DIR, LUA_LIBRARIES)
    find_package(Lua REQUIRED)
    message(STATUS "Found Lua via find_package: ${LUA_VERSION_STRING}")
    
    if(DEFINED LUA_INCLUDE_DIR)
        target_include_directories(sim_main PRIVATE ${LUA_INCLUDE_DIR})
    endif()
    
    if(DEFINED LUA_LIBRARIES)
        target_link_libraries(sim_main PRIVATE ${LUA_LIBRARIES})
    endif()
    
else()
    # Unix: Use pkg-config (try 5.4 first, fallback to 5.3)
    find_package(PkgConfig REQUIRED)
    
    pkg_check_modules(LUA54 QUIET lua5.4)
    pkg_check_modules(LUA53 QUIET lua5.3)
    
    if(LUA54_FOUND)
        target_include_directories(sim_main PRIVATE ${LUA54_INCLUDE_DIRS})
        target_link_libraries(sim_main PRIVATE ${LUA54_LIBRARIES})
        target_compile_options(sim_main PRIVATE ${LUA54_CFLAGS_OTHER})
        message(STATUS "Found Lua 5.4 via pkg-config")
    elseif(LUA53_FOUND)
        target_include_directories(sim_main PRIVATE ${LUA53_INCLUDE_DIRS})
        target_link_libraries(sim_main PRIVATE ${LUA53_LIBRARIES})
        target_compile_options(sim_main PRIVATE ${LUA53_CFLAGS_OTHER})
        message(STATUS "Found Lua 5.3 via pkg-config")
    else()
        message(FATAL_ERROR 
            "Lua not found via pkg-config (tried lua5.4 and lua5.3). "
            "Install liblua5.4-dev or liblua5.3-dev:\n"
            "  Ubuntu/Debian: sudo apt-get install liblua5.4-dev\n"
            "  Fedora: sudo dnf install lua-devel"
        )
    endif()
endif()

# ============================================================================
# Link Libraries
# ============================================================================
# Dependency graph:
#   sim_main → plant → utils (includes libcurl for InfluxDB)
#   sim_main → config (YAML vehicle loading)
# ============================================================================

target_link_libraries(sim_main PRIVATE
    plant   # Plant model and subsystems
    config  # Vehicle configuration loader (YAML)
    utils   # Logging, bitpacking, InfluxDB (includes libcurl)
)

# ============================================================================
# Compiler Features
# ============================================================================

target_compile_features(sim_main PRIVATE cxx_std_17)

# ============================================================================
# Installation
# ============================================================================

install(TARGETS sim_main
    RUNTIME DESTINATION bin
)

# ============================================================================
# Configuration Summary
# ============================================================================

message(STATUS "Simulation executable configured:")
message(STATUS "  - Target: sim_main")
message(STATUS "  - Dependencies: plant, config, utils, Lua")
message(STATUS "  - CAN sources: ${CAN_SOURCES}")
message(STATUS "  - Future: Move CAN sources to separate library")