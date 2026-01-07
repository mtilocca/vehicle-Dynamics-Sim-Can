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
add_subdirectory(src/config)     # NEW: Config library (depends on plant, utils)
add_subdirectory(src/sim)
add_subdirectory(src/can)

# Optional: Tests
include(CTest)  # provides BUILD_TESTING option
if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    enable_testing()
    add_subdirectory(test)
endif()
