// src/plant/physics_subsystem.hpp
#pragma once

#include "plant/plant_state.hpp"
#include "sim/actuator_cmd.hpp"

namespace plant {

/**
 * PhysicsSubsystem - Base class for all plant subsystems
 * 
 * This interface enables:
 * - Explicit dependency management
 * - Dynamic enable/disable of subsystems
 * - Multi-phase execution (pre/step/post)
 * - Scalable architecture (10+ subsystems)
 * 
 * Execution Order:
 *   1. initialize() - One-time setup
 *   2. pre_step()   - Prepare for timestep
 *   3. step()       - Main physics update
 *   4. post_step()  - Finalize/cleanup
 * 
 * Example Subsystems:
 * - SteerSubsystem (steering geometry)
 * - DriveSubsystem (longitudinal dynamics)
 * - BatterySubsystem (energy storage)
 * - ThermalSubsystem (temperature management)
 * - TireSubsystem (contact patch dynamics)
 * - AeroSubsystem (drag/downforce)
 */
class PhysicsSubsystem {
public:
    virtual ~PhysicsSubsystem() = default;

    // ========================================================================
    // Lifecycle Management
    // ========================================================================

    /**
     * initialize() - One-time setup when subsystem is registered
     * 
     * Use for:
     * - Setting initial state values
     * - Allocating resources
     * - Validating parameters
     * 
     * Called once by SubsystemManager after registration.
     */
    virtual void initialize(PlantState& s) {
        (void)s; // Default: no-op
    }

    /**
     * reset() - Reset subsystem to initial conditions
     * 
     * Use for:
     * - Resetting accumulated state (integrators, filters)
     * - Clearing fault flags
     * - Returning to known-good state
     * 
     * Called explicitly by user or on fault recovery.
     */
    virtual void reset(PlantState& s) {
        (void)s; // Default: no-op
    }

    // ========================================================================
    // Execution Phases
    // ========================================================================

    /**
     * pre_step() - Prepare for timestep (read dependencies, validate inputs)
     * 
     * Use for:
     * - Reading outputs from other subsystems
     * - Input validation/saturation
     * - Precalculations
     * 
     * Example:
     *   DriveSubsystem::pre_step() reads BatterySubsystem::get_available_power()
     * 
     * Execution: Before step(), every timestep
     */
    virtual void pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
        (void)s; (void)cmd; (void)dt; // Default: no-op
    }

    /**
     * step() - Main physics update (REQUIRED)
     * 
     * Use for:
     * - Integrating state variables
     * - Applying control commands
     * - Computing forces/torques
     * - Updating PlantState
     * 
     * Execution: Every timestep, after pre_step()
     */
    virtual void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) = 0;

    /**
     * post_step() - Finalize timestep (publish outputs, diagnostics)
     * 
     * Use for:
     * - Publishing computed values for other subsystems
     * - Diagnostic calculations
     * - Constraint enforcement
     * 
     * Example:
     *   BatterySubsystem::post_step() publishes SOC for next timestep
     * 
     * Execution: After step(), every timestep
     */
    virtual void post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
        (void)s; (void)cmd; (void)dt; // Default: no-op
    }

    // ========================================================================
    // Metadata & Control
    // ========================================================================

    /**
     * name() - Subsystem identifier for logging/debugging
     */
    virtual const char* name() const = 0;

    /**
     * enabled() - Check if subsystem is active
     * 
     * Disabled subsystems skip all execution phases.
     * Useful for lightweight testing or fault isolation.
     */
    virtual bool enabled() const { return enabled_; }

    /**
     * set_enabled() - Enable/disable subsystem
     * 
     * Example:
     *   thermal_subsystem->set_enabled(false); // Disable for fast testing
     */
    virtual void set_enabled(bool enabled) { enabled_ = enabled; }

    /**
     * priority() - Execution order hint (lower = earlier)
     * 
     * Used by SubsystemManager for ordering.
     * Default: 100 (normal priority)
     * 
     * Suggested priorities:
     *   0-49:   Inputs/sensors (read external data)
     *   50-99:  Actuators (steering, braking)
     *   100-149: Dynamics (forces, motion)
     *   150-199: Energy (battery, thermal)
     *   200+:    Outputs (diagnostics, logging)
     */
    virtual int priority() const { return 100; }

protected:
    bool enabled_ = true;
};

} // namespace plant
