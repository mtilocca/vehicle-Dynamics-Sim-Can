// src/plant/subsystem_manager.cpp
#include "plant/subsystem_manager.hpp"
#include "utils/logging.hpp"
#include <cstring>

namespace plant {

void SubsystemManager::register_subsystem(std::unique_ptr<PhysicsSubsystem> subsystem) {
    if (!subsystem) {
        LOG_WARN("[SubsystemManager] Attempted to register null subsystem");
        return;
    }

    const char* name = subsystem->name();
    const int priority = subsystem->priority();
    
    LOG_INFO("[SubsystemManager] Registering subsystem: %s (priority %d)", name, priority);

    subsystems_.push_back(std::move(subsystem));
    
    // Sort after each registration to maintain priority order
    sort_by_priority();
}

void SubsystemManager::initialize_all(PlantState& s) {
    LOG_INFO("[SubsystemManager] Initializing %zu subsystems", subsystems_.size());

    for (auto& subsystem : subsystems_) {
        if (subsystem->enabled()) {
            LOG_DEBUG("[SubsystemManager] Initializing: %s", subsystem->name());
            subsystem->initialize(s);
        }
    }
}

void SubsystemManager::reset_all(PlantState& s) {
    LOG_INFO("[SubsystemManager] Resetting %zu subsystems", subsystems_.size());

    for (auto& subsystem : subsystems_) {
        if (subsystem->enabled()) {
            LOG_DEBUG("[SubsystemManager] Resetting: %s", subsystem->name());
            subsystem->reset(s);
        }
    }
}

void SubsystemManager::step_all(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    // Phase 1: pre_step (read dependencies, validate inputs)
    for (auto& subsystem : subsystems_) {
        if (subsystem->enabled()) {
            subsystem->pre_step(s, cmd, dt);
        }
    }

    // Phase 2: step (main physics update)
    for (auto& subsystem : subsystems_) {
        if (subsystem->enabled()) {
            subsystem->step(s, cmd, dt);
        }
    }

    // Phase 3: post_step (publish outputs, diagnostics)
    for (auto& subsystem : subsystems_) {
        if (subsystem->enabled()) {
            subsystem->post_step(s, cmd, dt);
        }
    }
}

PhysicsSubsystem* SubsystemManager::find_subsystem(const char* name) {
    if (!name) return nullptr;

    for (auto& subsystem : subsystems_) {
        if (std::strcmp(subsystem->name(), name) == 0) {
            return subsystem.get();
        }
    }

    return nullptr;
}

PhysicsSubsystem* SubsystemManager::get_subsystem(size_t index) {
    if (index >= subsystems_.size()) {
        return nullptr;
    }
    return subsystems_[index].get();
}

size_t SubsystemManager::enabled_count() const {
    size_t count = 0;
    for (const auto& subsystem : subsystems_) {
        if (subsystem->enabled()) {
            ++count;
        }
    }
    return count;
}

void SubsystemManager::sort_by_priority() {
    std::sort(subsystems_.begin(), subsystems_.end(),
              [](const std::unique_ptr<PhysicsSubsystem>& a,
                 const std::unique_ptr<PhysicsSubsystem>& b) {
                  return a->priority() < b->priority();
              });

    // Log sorted order (debug only)
    LOG_DEBUG("[SubsystemManager] Execution order:");
    for (size_t i = 0; i < subsystems_.size(); ++i) {
        LOG_DEBUG("  %zu. %s (priority %d)", 
                  i + 1, subsystems_[i]->name(), subsystems_[i]->priority());
    }
}

} // namespace plant
