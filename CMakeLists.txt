cmake_minimum_required(VERSION 3.16)
project(vehicle_dynamics_sim_can VERSION 1.0 LANGUAGES CXX)

# ============================================================================
# Project Configuration
# ============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # For IDE support (clangd, etc.)

# ============================================================================
# Build Type Configuration
# ============================================================================

# Set default build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING 
        "Choose build type: Debug Release RelWithDebInfo MinSizeRel" 
        FORCE
    )
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ============================================================================
# Compiler Warnings and Flags
# ============================================================================

if(MSVC)
    # MSVC compiler warnings
    add_compile_options(/W4 /permissive-)
    
    # Suppress specific MSVC warnings if needed
    # add_compile_options(/wd4244)  # Example: disable C4244 (conversion warning)
else()
    # GCC/Clang compiler warnings
    add_compile_options(
        -Wall                # Enable most warnings
        -Wextra              # Enable extra warnings
        -Wpedantic           # Strict ISO C++ compliance
        -Wno-unused-parameter  # Suppress unused parameter warnings (common in callbacks)
    )
    
    # Additional flags for Debug builds
    if(CMAKE_BUILD_TYPE MATCHES Debug)
        add_compile_options(-g -O0)  # Full debug info, no optimization
    endif()
    
    # Additional flags for Release builds
    if(CMAKE_BUILD_TYPE MATCHES Release)
        add_compile_options(-O3 -DNDEBUG)  # Max optimization, disable asserts
    endif()
endif()

# ============================================================================
# Global Include Directories
# ============================================================================

include_directories(
    ${PROJECT_SOURCE_DIR}/src    # Allow includes like "plant/plant_model.hpp"
)

# ============================================================================
# Subdirectories - Build Order Matters (Dependencies)
# ============================================================================
# Build order is crucial due to library dependencies:
#   1. utils      (no dependencies)
#   2. plant      (depends on utils)
#   3. config     (depends on plant, utils, yaml-cpp)
#   4. sim        (depends on plant, config, utils, Lua)
#   5. can        (utility tools, depends on utils)
#   6. test       (depends on all of the above)
# ============================================================================

message(STATUS "=== Building Core Libraries ===")

# 1. Utils library (logging, bitpacking, InfluxDB client)
add_subdirectory(utils)

# 2. Plant library (vehicle dynamics and subsystems)
add_subdirectory(src/plant)

# 3. Config library (YAML vehicle configuration)
add_subdirectory(src/config)

# 4. Simulation executable
add_subdirectory(src/sim)

# 5. CAN utility tools (vcan_listener, vcan_random_sender)
add_subdirectory(src/can)

# ============================================================================
# Testing Configuration
# ============================================================================

# CTest provides BUILD_TESTING option (defaults to ON)
include(CTest)

if(BUILD_TESTING)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
        enable_testing()
        add_subdirectory(test)
        message(STATUS "=== Testing Enabled ===")
        message(STATUS "  - Test directory: ${CMAKE_CURRENT_SOURCE_DIR}/test")
        message(STATUS "  - Run tests with: ctest --test-dir ${CMAKE_BINARY_DIR}")
    else()
        message(WARNING "BUILD_TESTING is ON but test/ directory not found")
    endif()
else()
    message(STATUS "Testing disabled (BUILD_TESTING=OFF)")
endif()

# ============================================================================
# Installation Configuration
# ============================================================================

# Install binaries to bin/
install(TARGETS sim_main
    RUNTIME DESTINATION bin
)

# Install configuration files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING 
        PATTERN "*.yaml"
        PATTERN "*.json"
        PATTERN "*.csv"
        PATTERN "*.sh"
)

# ============================================================================
# Project Summary
# ============================================================================

message(STATUS "=== Project Configuration Summary ===")
message(STATUS "  Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  Testing: ${BUILD_TESTING}")
message(STATUS "")
message(STATUS "=== Build Targets ===")
message(STATUS "  Libraries: utils, plant, config")
message(STATUS "  Executables: sim_main, vcan_listener, vcan_random_sender")
message(STATUS "  Tests: 6 test executables (if BUILD_TESTING=ON)")
message(STATUS "")
message(STATUS "=== Build Commands ===")
message(STATUS "  Configure: cmake -S . -B build -DCMAKE_BUILD_TYPE=Release")
message(STATUS "  Build: cmake --build build -j$(nproc)")
message(STATUS "  Test: ctest --test-dir build")
message(STATUS "  Install: cmake --install build --prefix /usr/local")
message(STATUS "")