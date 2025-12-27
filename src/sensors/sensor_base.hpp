// src/sensors/sensor_base.hpp
#pragma once

#include "plant/plant_state.hpp"
#include "sensors/sensor_out.hpp"
#include <string>

namespace sensors {

/**
 * SensorBase - Abstract interface for all sensors
 * 
 * All sensors follow this lifecycle:
 * 1. Construction with parameters
 * 2. step(t, truth_state) → updates internal state, applies noise
 * 3. get_output() → returns measured values
 */
class SensorBase {
public:
    virtual ~SensorBase() = default;

    /**
     * Update sensor with new truth data
     * 
     * @param t Current simulation time (seconds)
     * @param truth Ground truth plant state
     * @param dt Timestep (seconds)
     */
    virtual void step(double t, const plant::PlantState& truth, double dt) = 0;

    /**
     * Get sensor measurements
     */
    virtual SensorOut get_output() const = 0;

    /**
     * Reset sensor to initial state
     */
    virtual void reset() = 0;

    /**
     * Get sensor name/ID
     */
    virtual std::string name() const = 0;

    /**
     * Check if sensor has valid data
     */
    virtual bool is_valid() const { return true; }
};

} // namespace sensors