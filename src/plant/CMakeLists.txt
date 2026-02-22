cmake_minimum_required(VERSION 3.16)

# ============================================================================
# Plant Library - Hybrid Torque-Slip Model Architecture
# ============================================================================
# 
# Execution Order (by subsystem priority):
#   50  - SteerSubsystem    : Ackermann steering geometry
#   100 - DriveSubsystem    : Powertrain torque distribution
#   105 - WheelSubsystem    : Wheel dynamics + tire forces (Dugoff)
#   110 - VehicleSubsystem  : Longitudinal/yaw/position integration
#
# Directory Structure:
#   plant_main/         - Core interfaces (PlantState, PhysicsSubsystem)
#   drive_subsystem/    - Drivetrain and torque distribution
#   steer_subsystem/    - Steering kinematics and Ackermann geometry
#   vehicle_subsystem/  - Vehicle longitudinal, yaw, position integration
#   subsystem_manager/  - Subsystem orchestration and lifecycle
# ============================================================================

# ----------------------------------------------------------------------------
# Collect all source files by subsystem
# ----------------------------------------------------------------------------

set(PLANT_SOURCES
    # Top-level plant model orchestration
    plant_model.cpp

    # Drive subsystem (Priority 100) — torque→force, linear cornering, wheel speeds
    drive_subsystem/drive_plant.cpp
    drive_subsystem/drive_subsystem.cpp

    # Steer subsystem (Priority 50) — Ackermann geometry, rate limiting
    steer_subsystem/steer_plant.cpp
    steer_subsystem/steer_subsystem.cpp

    # Vehicle subsystem (Priority 110) — 3-DOF rigid body integration
    vehicle_subsystem/vehicle_subsystem.cpp

    # Subsystem manager — priority-based execution
    subsystem_manager/subsystem_manager.cpp
)

# ----------------------------------------------------------------------------
# Create plant library
# ----------------------------------------------------------------------------

add_library(plant STATIC ${PLANT_SOURCES})

# ----------------------------------------------------------------------------
# Include directories
# ----------------------------------------------------------------------------

target_include_directories(plant PUBLIC
    ${CMAKE_SOURCE_DIR}           # For absolute paths: "src/plant/..."
    ${CMAKE_SOURCE_DIR}/src       # For relative paths: "plant/..."
)

# Make subsystem headers accessible to each other
# Allows: #include "battery_subsystem/battery_plant.hpp"
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
# Configuration summary
# ----------------------------------------------------------------------------

message(STATUS "Plant Library configured — subsystems Priority: Steer(50) Drive(100) Vehicle(110)")