cmake_minimum_required(VERSION 3.16)
project(plant_sensor_can_sim VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# Build Configuration
# ============================================================================

# Compiler warnings (compiler-specific)
if (MSVC)
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/src
)

# ============================================================================
# Subdirectories
# ============================================================================

# Core libraries (build order matters due to dependencies)
add_subdirectory(utils)
add_subdirectory(src/plant)
add_subdirectory(src/config)     # Config library (depends on plant, utils)
add_subdirectory(src/sim)
add_subdirectory(src/can)

# ============================================================================
# Testing
# ============================================================================

# Enable CTest support
include(CTest)  # Provides BUILD_TESTING option (defaults to ON)

# Add test directory if it exists and BUILD_TESTING is ON
if(BUILD_TESTING)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
        enable_testing()
        add_subdirectory(test)
    else()
        message(WARNING "BUILD_TESTING is ON but test/ directory not found")
    endif()
endif()