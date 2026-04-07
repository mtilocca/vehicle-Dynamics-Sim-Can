cmake_minimum_required(VERSION 3.16)

# ============================================================================
# Configuration library
# ============================================================================

add_library(config
    vehicle_config.cpp
)

target_include_directories(config PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

# ============================================================================
# Dependencies
# ============================================================================

# Link to utils for logging
target_link_libraries(config PUBLIC utils)

# Link to plant for PlantModelParams
target_link_libraries(config PUBLIC plant)
