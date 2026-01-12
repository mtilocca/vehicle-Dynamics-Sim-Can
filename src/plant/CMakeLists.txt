cmake_minimum_required(VERSION 3.16)

# ============================================================================
# Plant Library - Reorganized Subsystem Architecture
# ============================================================================
# Structure:
#   plant_main/       - Core interfaces and bicycle kinematics
#   battery_subsystem/ - Battery physics and energy management
#   drive_subsystem/   - Drivetrain and longitudinal dynamics
#   steer_subsystem/   - Steering kinematics and Ackermann geometry
#   tyre_subsystem/    - Tire models (future: Dugoff, Pacejka)
#   thermal_subsystem/ - Thermal management (future)
#   subsystem_manager/ - Subsystem orchestration and lifecycle
# ============================================================================

# ----------------------------------------------------------------------------
# Collect all source files by subsystem
# ----------------------------------------------------------------------------

set(PLANT_SOURCES
    # Top-level plant model orchestration
    plant_model.cpp
    
    # Plant main - core interfaces and kinematics
    plant_main/vehicle_bicycle_ackermann.cpp
    
    # Battery subsystem
    battery_subsystem/battery_plant.cpp
    battery_subsystem/battery_subsystem.cpp
    
    # Drive subsystem
    drive_subsystem/drive_plant.cpp
    drive_subsystem/drive_subsystem.cpp
    
    # Steer subsystem
    steer_subsystem/steer_plant.cpp
    steer_subsystem/steer_subsystem.cpp
    
    # Subsystem manager
    subsystem_manager/subsystem_manager.cpp
    
    # Future subsystems (add when implemented):
    # tyre_subsystem/tyre_dugoff.cpp
    # tyre_subsystem/tyre_subsystem.cpp
    # thermal_subsystem/thermal_plant.cpp
    # thermal_subsystem/thermal_subsystem.cpp
)

# ----------------------------------------------------------------------------
# Create plant library
# ----------------------------------------------------------------------------

add_library(plant STATIC ${PLANT_SOURCES})

# ----------------------------------------------------------------------------
# Include directories
# ----------------------------------------------------------------------------

target_include_directories(plant PUBLIC
    ${CMAKE_SOURCE_DIR}           # For absolute paths like "src/plant/..."
    ${CMAKE_SOURCE_DIR}/src       # For relative paths like "plant/..."
)

# Make subsystem headers accessible to each other
# This allows: #include "battery_subsystem/battery_plant.hpp"
target_include_directories(plant PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}   # src/plant directory
)

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------

target_link_libraries(plant PUBLIC 
    utils  # Logging, bitpacking, InfluxDB client
)

# ----------------------------------------------------------------------------
# Compiler features
# ----------------------------------------------------------------------------

target_compile_features(plant PUBLIC cxx_std_17)

# ----------------------------------------------------------------------------
# Optional: Print configuration summary
# ----------------------------------------------------------------------------

message(STATUS "Plant library configured:")
message(STATUS "  - Subsystems: Battery, Drive, Steer")
message(STATUS "  - Future: Tyre, Thermal")
message(STATUS "  - Source files: ${PLANT_SOURCES}")