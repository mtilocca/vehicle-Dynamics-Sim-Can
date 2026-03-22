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
#   150 - BatterySubsystem  : Energy management
#
# Directory Structure:
#   plant_main/         - Core interfaces (PlantState, PhysicsSubsystem)
#   battery_subsystem/  - Battery physics and energy management
#   drive_subsystem/    - Drivetrain and torque distribution
#   steer_subsystem/    - Steering kinematics and Ackermann geometry
#   wheel_subsystem/    - Wheel rotational dynamics and tire slip
#   tyre_subsystem/     - Tire force models (Dugoff, future: Pacejka)
#   vehicle_subsystem/  - Vehicle longitudinal, yaw, position integration
#   subsystem_manager/  - Subsystem orchestration and lifecycle
#
# Future Subsystems:
#   thermal_subsystem/  - Thermal management (battery, motor, brakes)
#   aero_subsystem/     - Aerodynamic forces (drag, downforce)
#   suspension_subsystem/ - Load transfer, ride dynamics
# ============================================================================

# ----------------------------------------------------------------------------
# Collect all source files by subsystem
# ----------------------------------------------------------------------------

set(PLANT_SOURCES
    # Top-level plant model orchestration
    plant_model.cpp
    
    # Battery subsystem (Priority 150)
    battery_subsystem/battery_plant.cpp
    battery_subsystem/battery_subsystem.cpp
    
    # Drive subsystem (Priority 100)
    drive_subsystem/drive_plant.cpp
    drive_subsystem/drive_subsystem.cpp
    
    # Steer subsystem (Priority 50)
    steer_subsystem/steer_plant.cpp
    steer_subsystem/steer_subsystem.cpp
    
    # Wheel subsystem (Priority 105)
    # Integrates wheel rotational dynamics (Iw·ω̇ = τ - Fx·R)
    # Computes tire forces via Dugoff model
    wheel_subsystem/wheel_dynamics.cpp
    wheel_subsystem/wheel_subsystem.cpp
    
    # Tyre subsystem - Dugoff tire force model
    # Separable design: easy to swap with Pacejka, MF-Tyre, etc.
    tyre_models/tyre_dugoff.cpp
    
    # Vehicle subsystem (Priority 110)
    # Integrates longitudinal velocity, yaw angle, position
    # Replaces legacy VehicleBicycleAckermann
    vehicle_subsystem/vehicle_subsystem.cpp
    
    # Subsystem manager - Priority-based execution
    subsystem_manager/subsystem_manager.cpp
    
    # Future subsystems (add when implemented):
    # thermal_subsystem/thermal_plant.cpp
    # thermal_subsystem/thermal_subsystem.cpp
    # aero_subsystem/aero_plant.cpp
    # aero_subsystem/aero_subsystem.cpp
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

message(STATUS "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
message(STATUS "Plant Library - Hybrid Torque-Slip Model")
message(STATUS "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
message(STATUS "Subsystems (execution order):")
message(STATUS "  50  - Steer    : Ackermann geometry")
message(STATUS "  100 - Drive    : Torque distribution + regen")
message(STATUS "  105 - Wheel    : Rotational dynamics + Dugoff tire model")
message(STATUS "  110 - Vehicle  : Longitudinal/yaw/position integration")
message(STATUS "  150 - Battery  : Energy management")
message(STATUS "")
message(STATUS "Architecture:")
message(STATUS "  ✓ Hybrid torque-slip model (PDF Section 8)")
message(STATUS "  ✓ Closed-loop feedback: τ → ω → σx → Fx → v")
message(STATUS "  ✓ Modular tire models (Dugoff, future: Pacejka)")
message(STATUS "  ✓ Natural traction limiting via friction saturation")
message(STATUS "  ✗ Kinematic mode removed (dynamic only)")
message(STATUS "")
message(STATUS "Source files: ${PLANT_SOURCES}")
message(STATUS "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")