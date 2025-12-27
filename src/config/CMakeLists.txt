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

# yaml-cpp (try pkg-config first, then find_package)
find_package(PkgConfig QUIET)

if (PkgConfig_FOUND)
    pkg_check_modules(YAML_CPP QUIET yaml-cpp)
endif()

if (YAML_CPP_FOUND)
    # Found via pkg-config
    target_include_directories(config PRIVATE ${YAML_CPP_INCLUDE_DIRS})
    target_link_libraries(config PRIVATE ${YAML_CPP_LIBRARIES})
    target_compile_options(config PRIVATE ${YAML_CPP_CFLAGS_OTHER})
    message(STATUS "Found yaml-cpp via pkg-config: ${YAML_CPP_VERSION}")
else()
    # Try CMake find_package
    find_package(yaml-cpp REQUIRED)
    target_link_libraries(config PRIVATE yaml-cpp)
    message(STATUS "Found yaml-cpp via find_package")
endif()

# Link to utils for logging
target_link_libraries(config PUBLIC utils)

# Link to plant for PlantModelParams
target_link_libraries(config PUBLIC plant)