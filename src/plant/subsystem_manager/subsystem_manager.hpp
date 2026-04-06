// src/plant/subsystem_manager.hpp
#pragma once

#include "plant/plant_main/physics_subsystem.hpp"

#ifdef __ZEPHYR__
// ── Zephyr build: no heap, fixed-size raw-pointer array ──────────────────────
// PlantModel owns the subsystem instances as value members; this manager
// holds non-owning raw pointers sorted by priority (insertion sort).
#include <stddef.h>

namespace plant {

class SubsystemManager {
public:
    static constexpr size_t MAX_SUBSYSTEMS = 8;

    SubsystemManager() = default;
    SubsystemManager(const SubsystemManager&) = delete;
    SubsystemManager& operator=(const SubsystemManager&) = delete;

    // Register a non-owning pointer. Kept sorted by priority after each call.
    void register_subsystem(PhysicsSubsystem* s);

    void initialize_all(PlantState& s);
    void reset_all(PlantState& s);
    void step_all(PlantState& s, const sim::ActuatorCmd& cmd, double dt);

    PhysicsSubsystem* find_subsystem(const char* name);
    PhysicsSubsystem* get_subsystem(size_t index);
    size_t subsystem_count() const { return count_; }
    size_t enabled_count() const;

private:
    PhysicsSubsystem* subsystems_[MAX_SUBSYSTEMS]{};
    size_t count_ = 0;
    void sort_by_priority();
};

} // namespace plant

#else
// ── Host build: heap-allocated unique_ptr vector ─────────────────────────────
#include <memory>
#include <vector>
#include <algorithm>

namespace plant {

class SubsystemManager {
public:
    SubsystemManager() = default;
    ~SubsystemManager() = default;
    SubsystemManager(const SubsystemManager&) = delete;
    SubsystemManager& operator=(const SubsystemManager&) = delete;

    void register_subsystem(std::unique_ptr<PhysicsSubsystem> subsystem);
    void initialize_all(PlantState& s);
    void reset_all(PlantState& s);
    void step_all(PlantState& s, const sim::ActuatorCmd& cmd, double dt);
    PhysicsSubsystem* find_subsystem(const char* name);
    PhysicsSubsystem* get_subsystem(size_t index);
    size_t subsystem_count() const { return subsystems_.size(); }
    size_t enabled_count() const;

private:
    std::vector<std::unique_ptr<PhysicsSubsystem>> subsystems_;
    void sort_by_priority();
};

} // namespace plant
#endif // __ZEPHYR__
