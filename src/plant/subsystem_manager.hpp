// src/plant/subsystem_manager.hpp
#pragma once

#include "plant/physics_subsystem.hpp"
#include <memory>
#include <vector>
#include <algorithm>

namespace plant {

/**
 * SubsystemManager - Orchestrates execution of physics subsystems
 * 
 * Responsibilities:
 * - Register subsystems
 * - Order by priority
 * - Execute pre_step/step/post_step phases
 * - Handle enable/disable
 * 
 * Usage:
 *   SubsystemManager mgr;
 *   mgr.register_subsystem(std::make_unique<SteerSubsystem>());
 *   mgr.register_subsystem(std::make_unique<DriveSubsystem>());
 *   mgr.register_subsystem(std::make_unique<BatterySubsystem>());
 *   
 *   // Each timestep:
 *   mgr.step_all(state, cmd, dt);
 */
class SubsystemManager {
public:
    SubsystemManager() = default;
    ~SubsystemManager() = default;

    // Non-copyable (owns unique_ptr subsystems)
    SubsystemManager(const SubsystemManager&) = delete;
    SubsystemManager& operator=(const SubsystemManager&) = delete;

    // ========================================================================
    // Registration
    // ========================================================================

    /**
     * register_subsystem() - Add a subsystem to the manager
     * 
     * Subsystems are executed in priority order (lower = earlier).
     * Calls subsystem->initialize(state) after registration.
     * 
     * Example:
     *   mgr.register_subsystem(std::make_unique<SteerSubsystem>(params));
     */
    void register_subsystem(std::unique_ptr<PhysicsSubsystem> subsystem);

    /**
     * initialize_all() - Initialize all registered subsystems
     * 
     * Called automatically by register_subsystem() for each subsystem.
     * Can be called manually to re-initialize all subsystems.
     */
    void initialize_all(PlantState& s);

    /**
     * reset_all() - Reset all subsystems to initial conditions
     * 
     * Useful for:
     * - Starting a new scenario
     * - Recovering from faults
     * - Re-running tests
     */
    void reset_all(PlantState& s);

    // ========================================================================
    // Execution
    // ========================================================================

    /**
     * step_all() - Execute one timestep for all enabled subsystems
     * 
     * Execution order (by priority):
     *   1. pre_step()  for all enabled subsystems
     *   2. step()      for all enabled subsystems
     *   3. post_step() for all enabled subsystems
     * 
     * Disabled subsystems are skipped entirely.
     */
    void step_all(PlantState& s, const sim::ActuatorCmd& cmd, double dt);

    // ========================================================================
    // Query & Control
    // ========================================================================

    /**
     * find_subsystem() - Find subsystem by name
     * 
     * Returns nullptr if not found.
     * 
     * Example:
     *   auto* battery = mgr.find_subsystem("Battery");
     *   if (battery) battery->set_enabled(false);
     */
    PhysicsSubsystem* find_subsystem(const char* name);

    /**
     * get_subsystem() - Get subsystem by index
     * 
     * Returns nullptr if index out of bounds.
     */
    PhysicsSubsystem* get_subsystem(size_t index);

    /**
     * subsystem_count() - Number of registered subsystems
     */
    size_t subsystem_count() const { return subsystems_.size(); }

    /**
     * enabled_count() - Number of enabled subsystems
     */
    size_t enabled_count() const;

private:
    std::vector<std::unique_ptr<PhysicsSubsystem>> subsystems_;

    // Helper: Sort subsystems by priority
    void sort_by_priority();
};

} // namespace plant
